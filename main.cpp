/**
 * Simple program for sniffing RF packets from Current Cost transmitters and
 * sending each packet to the host computer, unprocessed.
 *
 *
 * Hardware requirements:
 *
 *     - An ATmega processor (I used a Nanode but I expect an Arduino or
 *       JeeNode or DIY ATmega board will work fine).
 *
 *     - An RFM01 433Mhz receiver module (about £6 from uk.farnell.com)
 *
 *     - Make sure the RFM01's nFFS pin is set permanently high.
 *
 *
 * Usage:
 *
 *     - Load this program onto your ATmega.  (If you're using an Arduino IDE
 *       then I think all you have to do it copy all this code and delete
 *       the main() function.  I haven't tried this using the Arduino IDE though.)
 *
 *     - Then watch the serial port.  On Linux this can be achieved with
 *       the command:
 *           screen /dev/ttyUSB0 115200
 *
 *     - You should see each RF packet displayed on your screen (each byte
 *       displayed as hex).
 *
 *
 * Acknowledgements & further reading
 *
 *     - Many thanks to the good folks who wrote the JeeLib as it provided lots
 *       of help.
 *
 *     - As did the following tutorial on SPI on AVR devices:
 *       https://sites.google.com/site/qeewiki/books/avr-guide/spi
 *
 *     - If you want an indepth insight into the RFM01 then read the RFM01.pdf
 *       manual BEFORE RFM01_code.pdf as the latter has far less detail.
 *       The only advantage of the RFM01_code.pdf is that it contains a
 *       code example in C.
 *
 *     - I haven't attempted to decode the packets yet.  Apparently these
 *       packets are "manchesterised" and may be reversed.  See
 *
 *       http://jack-kelly.com/sniffing_spi_data_from_my_current_cost_envir
 *          and
 *       http://gangliontwitch.com/ccPower.html
 */

#include <Arduino.h>

// Pins
#define RFM_IRQ      2 // PD2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT       2 // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_SS      10 // PB2, pin 16 (confirmed from Nanode RF schematic)
#define SPI_MOSI    11 // PB3, pin 17
#define SPI_MISO    12 // PB4, pin 18
#define SPI_SCK     13 // PB5, pin 19

/**
 * Simple class for representing a packet of consecutive bytes
 * received from a Current Cost transmitter.
 */
class Packet {
public:
	Packet(): bytes_read(0) {}

	/**
	 * Add a byte to the packet.
	 */
	volatile void add(const uint8_t value) {
		if (bytes_read < PACKET_SIZE) {
			packet[bytes_read++] = value;
		}
	}

	void print() const {
		for (int i=0; i<PACKET_SIZE; i++) {
			Serial.print(packet[i], HEX);
			Serial.print(" ");
		}
		Serial.print("\r\n");
	}

	bool full() {
		return bytes_read == PACKET_SIZE;
	}

	volatile void reset() {
		bytes_read = 0;
	}

private:
	static const uint8_t PACKET_SIZE = 16; // number of bytes in a packet
	volatile uint8_t bytes_read; // number of bytes read so far
	volatile uint8_t packet[PACKET_SIZE];
};

/**
 * Class for storing multiple packets.  We need this because
 * multiple packets might arrive before we have a chance to
 * read these packets over the FTDI serial port.
 */
class PacketBuffer {
public:
	// FIXME: concurrency issues. Research mutexes on Arduino.

	PacketBuffer(): current_packet(0) {}

	/**
	 * @returns true if packet is complete AFTER adding value to it.
	 */
	volatile const bool add(const uint8_t value) {
		packets[current_packet].add(value);

		if (packets[current_packet].full()) {
			if (current_packet >= NUM_PACKET) {
				Serial.println("NO MORE BUFFERS!");
			} else {
				current_packet++;
			}
			return true;
		} else {
			return false;
		}
	}

	void print_and_reset() {
		for (int i=0; i<current_packet; i++) {
			packets[i].print();
			packets[i].reset();
		}
		if (packets[current_packet].full()) {
			packets[current_packet].print();
			packets[current_packet].reset();
		}
		current_packet = 0;
	}

	volatile const bool data_is_available() const {
		return current_packet > 0;
	}

private:
	static const uint8_t NUM_PACKET = 5;
	uint8_t current_packet;
	Packet packets[NUM_PACKET];
};

PacketBuffer PACKET_BUFFER;


/**
 * Select or de-select the RFM01.
 *
 * @param state: if true then select the RFM01 (ready for data transfer)
 */
inline void spi_select(const bool state)
{
	if (state == true) {
		bitClear(SS_PORT, SS_BIT);
	} else {
		bitSet(SS_PORT, SS_BIT);
	}
}

/**
 * Initialise the ATmega's SPI hardware for use with the RFM01.
 */
void spi_init()
{
	spi_select(false);
    bitSet(SS_DDR, SS_BIT);
    digitalWrite(SPI_SS, 1);

    pinMode(SPI_SS, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(SPI_SCK, OUTPUT);

    // SPE enables SPI
    // MSTR instructs AVR to operate in SPI master mode
    SPCR = _BV(SPE) | _BV(MSTR);

    pinMode(RFM_IRQ, INPUT);
    digitalWrite(RFM_IRQ, 1); // pull-up

	// RFM12b can cope with a 20MHz SPI clock. Page 13 of the RFM12b manual
	// gives clock high time and clock low time as 25ns each. Hence total
	// cycle time is 50ns. Hence max frequency is 1/( 50 x 10^-9 ).
	// DIV2 is the fastest we can go.
    // https://sites.google.com/site/qeewiki/books/avr-guide/spi
    // But jeelib uses clk/8 for sending so let's use that
    SPSR |= _BV(SPI2X);
    bitClear(SPCR, SPR1);
	bitSet(SPCR, SPR0);

	// CPOL=0 (base value of clock is zero)
	// CPHA=0 (data bits read upon rising edge of clock)
	bitClear(SPCR, CPOL);
	bitClear(SPCR, CPHA);
}

/**
 * Send a byte over the SPI bus
 *
 * @return 8-bit response
 */
static const uint8_t spi_transfer_byte(const uint8_t& out)
{
	SPDR = out;
    // this loop spins 4 usec with a 2 MHz SPI clock
    while (!(SPSR & _BV(SPIF)))
        ; // SPIF goes high when data transfer complete
    return SPDR;
}

/**
 * Send a 16-bit word over the SPI bus, with MSB first.
 *
 * @return 16-bit response. First response is MSB.
 */
static const uint16_t spi_transfer_word(const uint16_t& cmd, const bool& select = true)
{
	if (select) spi_select(true);
	uint16_t reply = spi_transfer_byte(cmd >> 8) << 8; 	// transfer MSB first
	reply |= spi_transfer_byte(cmd & 0x00FF); // transfer LSB
	if (select) spi_select(false);
	return reply;
}

void reset_FIFO_RFM01()
{
    // From RFM01 command #12 CE89 (11. output and FIFO mode) (gangliontwitch has CE88)
    // f3 f2 f1 f0 s1 s0 ff fe
    //  1  0  0  0  1  0  0  0
    // f: FIFO interrupt level = 8
    // s: FIFO fill start condition = reserved
    // ff=0: disable FIFO fill
    // fe=0: disable FIFO function
    spi_transfer_word(0xCE88);

    // From RFM01 command #14 CE8B (11. output and FIFO mode)
    // f3 f2 f1 f0 s1 s0 ff fe
    //  1  0  0  0  1  0  1  1
    // f: FIFO interrupt level = 8
    // s: FIFO fill start condition = reserved
    // ff=1: enable FIFO fill
    // fe=1: enable FIFO function
   spi_transfer_word(0xCE8B);
}

static void rfm01_interrupt()
{
	spi_select(true);
	bool full = false; // is the buffer full after receiving the byte waiting for us?
	const uint8_t status_MSB = spi_transfer_byte(0x00); // get status word MSB
	const uint8_t status_LSB = spi_transfer_byte(0x00); // get status word LSB
	const uint8_t data_1     = spi_transfer_byte(0x00); // get 1st byte of data

	if ((status_MSB & 0x40) != 0) { // FIFO overflow
		full  = PACKET_BUFFER.add(data_1);
		full |= PACKET_BUFFER.add(spi_transfer_byte(0x00));
	} else 	if ((status_MSB & 0x80) != 0) { // FIFO has 8 bits ready
		full = PACKET_BUFFER.add(data_1);
	}
	spi_select(false);

	if (full) {
		reset_FIFO_RFM01();
	}
}

/**
 * These initialisation commands are almost all taken
 * directly from sniffing the SPI bus of a Current Cost EnviR v1.29.
 *
 * http://jack-kelly.com/sniffing_spi_data_from_my_current_cost_envir
 *
 * I have done my best to interpret these commands using the RFM01.pdf manual.
 */
void rf01_init_cc ()
{
	Serial.println("Starting rf01_initialize_cc() ");

    spi_init();

	spi_transfer_word(0x0000);
	spi_transfer_word(0xFF00); // software reset command (p22 of RFM01.pdf)

    delay(2000); // give RFM time to start up

    Serial.println("RFM01 finished power-up reset.  Starting init...");

    // From RFM01 command #1 0x892D
    // eb=0 (disable low batt detection)
    // et=0 (disable wake-up timer)
    // ex=1 (enable crystal oscillator)
    // baseband bandwidth = 67kHz
    // dc=1 (disable signal output of CLK pin)
    spi_transfer_word(0x892D);

    // RFM01 command #2 E196 (5. wake-up timer command)
    spi_transfer_word(0xE196);

    // RFM01 command #3 CC0E (6. low duty-cycle command)
    // en = 0: disable low duty cycle mode
    spi_transfer_word(0xCC0E);

    // From RFM01 command #4 C69F (8. AFC Command)
    // a1 a0 rl1 rl0 st fi oe en
    //  1  0   0   1  1  1  1  1
    // a  = AFC auto-mode: keep offset when VDI hi
    // rl = range limit: +15/-16 (433band: 2.5kHz)
    // st=1 st goes hi will store offset into output register
    // fi=1 Enable AFC hi accuracy mode
    // oe=1 Enable AFC output register
    // en=1 Enable AFC function
    spi_transfer_word(0xC69F);

    // From RFM01 command #5 C46A (9. data filter command)
    // al ml 1 s1 s0 f2 f1 f0
    //  0  1 1  0  1  0  1  0
    // al=0: disable clock recovery auto-lock
    // ml=1: enable clock recovery fast mode
    // s: data filter=digital filter
    // f: DQD threshold = 2
    spi_transfer_word(0xC46A);

    // From RFM command #6 C88A
    // 3918.5 bps
    spi_transfer_word(0xC88A);

    // From RFM01 command #7 C080 (4. receiver setting command)
    // d1 d0 g1 g0 r2 r1 r0 en
    //  1  0  0  0  0  0  0  0
    // d: VDI source = clock recovery lock output
    // g: LNA gain = 0 dBm
    // r: DRSSI threshold = -103 dBm
    // en=0: disable receiver
    spi_transfer_word(0xC080);

    // GanglionTwitch has command CE88 here, my CC doesn't (11. output and FIFO mode)

    // From RFM01 command #8 CE8B (11. output and FIFO mode)
    // f3 f2 f1 f0 s1 s0 ff fe
    //  1  0  0  0  1  0  1  1
    // f: FIFO interrupt level = 8
    // s: FIFO fill start condition = reserved
    // ff=1: enable FIFO fill
    // fe=1: enable FIFO function
    spi_transfer_word(0xCE8B);

    // From RFM01 command #9 C081 (4. receiver setting command)
    // d1 d0 g1 g0 r2 r1 r0 en
    //  1  0  0  0  0  0  0  1
    // d: VDI source = clock recovery lock output
    // g: LNA gain = 0 dBm
    // r: DRSSI threshold = -103 dBm
    // en=1: enable receiver <--- only diff from command #7
    spi_transfer_word(0xC081);

    // From RFM01 command #10 C200 (7. Low Batt Detector & MCU Clock Div)
    // d2 d1 d0 t4 t3 t2 t1 t0
    //  0  0  0  0  0  0  0  0
    // d: frequency of CLK pin = 1MHz
    // t: low batt detection theshold = 2.2+0 V
    spi_transfer_word(0xC200);

    // From RFM01 command #11 A618 (3. frequency setting command)
    // Fc = 433.9MHz
    spi_transfer_word(0xA618);

    // From RFM01 command #12 CE89 (11. output and FIFO mode) (gangliontwitch has CE88)
    // f3 f2 f1 f0 s1 s0 ff fe
    //  1  0  0  0  1  0  0  1
    // f: FIFO interrupt level = 8
    // s: FIFO fill start condition = reserved
    // ff=0: disable FIFO fill
    // fe=1: enable FIFO function
    spi_transfer_word(0xCE88);

    // From RFM01 command #14 CE8B (11. output and FIFO mode)
    // f3 f2 f1 f0 s1 s0 ff fe
    //  1  0  0  0  1  0  1  1
    // f: FIFO interrupt level = 8
    // s: FIFO fill start condition = reserved
    // ff=1: enable FIFO fill
    // fe=1: enable FIFO function
    spi_transfer_word(0xCE8B);

    // Reset Mode Command is not set so it defaults to DA00
    // (do not disable highly sensitive reset)
    spi_transfer_word(0xDA01); // try disabling because of weird behaviour

    spi_transfer_word(0x0000); // clear
    spi_transfer_word(0x0000); // clear

    Serial.println("attaching interrupt");
    delay(500);
    attachInterrupt(0, rfm01_interrupt, LOW);
    return;
}

void setup()
{
	Serial.begin(115200);
	Serial.println("Current Cost receiver");
	rf01_init_cc();
	Serial.println("Finished init.");
}

void loop()
{
	delay(1000);
	Serial.println(".");
	if (PACKET_BUFFER.data_is_available()) {
		PACKET_BUFFER.print_and_reset();
	}
}


int main(void)
{
  init();
  setup();

  while(true) {
    loop();
  }
}

