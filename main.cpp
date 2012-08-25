// Serial data over RF12 demo, works in both directions
// 2009-04-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <Arduino.h>
#include <SPI.h>

/**
 * TODO: Sniff SPI of IAM to find synchron bit sequence.  If that fails then
 *       order RFM01.
 */

#define SPI_SELECT  10 // PB2, pin16 (confirmed from Nanode RF schematic)
#define RFM_IRQ      2 // PD2
#define SS_PORT     PORTB
#define SS_BIT       2 // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8


void spi_select(const bool state)
{
	digitalWrite(SPI_SELECT, !state);
/*	if (state == 1) {
		bitClear(SS_PORT, SS_BIT);
	} else {
		bitSet(SS_PORT, SS_BIT);
	}
	*/
}

void spi_init()
{
	spi_select(false);
	pinMode(SPI_SELECT, OUTPUT);

	pinMode(RFM_IRQ, INPUT);
	digitalWrite(RFM_IRQ, 1); // pull-up

	SPI.begin();
	SPI.setBitOrder(MSBFIRST);

	// RFM12b can cope with a 20MHz SPI clock. Page 13 of the RFM12b manual
	// gives clock high time and clock low time as 25ns each. Hence total
	// cycle time is 50ns. Hence max frequency is 1/( 50 x 10^-9 ).
	// DIV2 is the fastest we can go.
	SPI.setClockDivider(SPI_CLOCK_DIV2);

	// CPOL=0 (base value of clock is zero)
	// CPHA=0 (data bits shifted into RFM12 upon rising edge of clock)
	SPI.setDataMode(SPI_MODE0);

}

/**
 * Send a 16-bit word over the SPI bus, with MSB first.
 *
 * @return 16-bit response. First response is MSB.
 */
static uint16_t rfm_transfer(const uint16_t cmd)
{
	spi_select(true);
	uint16_t reply = SPI.transfer(cmd >> 8) << 8; 	// transfer MSB first
	reply |= SPI.transfer(cmd & 0x00FF); // transfer LSB
	spi_select(false);
	return reply;
}

volatile byte bytes_read = 0;

void rfm12_interrupt()
{
	uint16_t irq_reason = rfm_transfer(0x0000); // read reason for IRQ

	if (irq_reason & 0x8000) { // FFIT: FIFO has 8 bits
		uint16_t reply = rfm_transfer(0xB000);
		Serial.print(reply & 0x00FF); // Read a byte
		Serial.print(" ");
		bytes_read++;
	} else if (irq_reason & 0x2000) { // FFOV: FIFO overflow: has 16 bits
		rfm_transfer(0xB000); // Read a byte
		rfm_transfer(0xB000); // Read a byte
		bytes_read += 2;
	}

	if (bytes_read >= 16) {
		bytes_read = 0;
		Serial.print("\r\n");

		// To restart synchron pattern recognition, bit ff should be cleared and set
		// i.e. packet will start with preable, then synchron pattern, then 16 bytes
		// of data, then turn FIFO off and on (I think)

		// 8. FIFO and Reset Mode Command
		// 1 1 0 0 1 0 1 0 f3 f2 f1 f0 sp al ff dr
		//
		// f: FIFO interrupt level = 8 (RFM01 & default)
		// sp: length of synchron pattern (not on RFM01!!!) <-- DEAL BREAKER FOR USING RFM12 with RFM01?!
		// al: FIFO fill start condition. Not on RFM01. Default = sync-word but RFM01 doesn't output sync word?
		//     0=synchron pattern
		//     1=always fill
		// ff: enable FIFO fill
		// dr: disable hi sensitivity reset mode
		//
		//                         safd
		//             11001010ffffplfr
		rfm_transfer(0b1100101010001000);

		// 8. FIFO and Reset Mode Command
		// 1 1 0 0 1 0 1 0 f3 f2 f1 f0 sp al ff dr
		//
		// f: FIFO interrupt level = 8 (RFM01 & default)
		// sp: length of synchron pattern (not on RFM01!!!) <-- DEAL BREAKER FOR USING RFM12 with RFM01?!
		// al: FIFO fill start condition. Not on RFM01. Default = sync-word but RFM01 doesn't output sync word?
		//     0=synchron pattern
		//     1=always fill
		// ff: enable FIFO fill
		// dr: disable hi sensitivity reset mode
		//
		//                         safd
		//             11001010ffffplfr
		rfm_transfer(0b1100101010001010);
	}
}

void rf12_init_cc () {
    Serial.println("Starting rf12_initialize_cc() ");

    spi_init();

	spi_select(true);
	rfm_transfer(0x0000);
	spi_select(false);

    delay(2000); // give RFM time to start up

    Serial.println("RFM12b finished power-up reset.  Starting init...");

     // From RFM01 command #1 0x892D
     // eb=0 (disable low batt detection)
     // et=0 (disable wake-up timer)
     // ex=1 (enable crystal oscillator)
     // baseband bandwidth = 67kHz
     // dc=1 (disable signal output of CLK pin)

     // RFM01 command #2 E196 (5. wake-up timer command)

     // RFM01 command #3 CC0E (6. low duty-cycle command)
     // en = 0: disable low duty cycle mode

     // From RFM01 command #4 C69F (8. AFC Command)
     // a1 a0 rl1 rl0 st fi oe en
     //  1  0   0   1  1  1  1  1
     // a  = AFC auto-mode: keep offset when VDI hi
     // rl = range limit: +15/-16 (433band: 2.5kHz)
     // st=1 st goes hi will store offset into output register
     // fi=1 Enable AFC hi accuracy mode
     // oe=1 Enable AFC output register
     // en=1 Enable AFC function

     // From RFM01 command #5 C46A (9. data filter command)
     // al ml 1 s1 s0 f2 f1 f0
     //  0  1 1  0  1  0  1  0
     // al=0: disable clock recovery auto-lock
     // ml=1: enable clock recovery fast mode
     // s: data filter=digital filter
     // f: DQD threshold = 2

     // From RFM command #6 C88A
     // 3918.5 bps

     // From RFM01 command #7 C080 (4. receiver setting command)
     // d1 d0 g1 g0 r2 r1 r0 en
     //  1  0  0  0  0  0  0  0
     // d: VDI source = clock recovery lock output
     // g: LNA gain = 0 dBm
     // r: DRSSI threshold = -103 dBm
     // en=0: disable receiver

     // GanglionTwitch has command CE88 here, my CC doesn't (11. output and FIFO mode)

     // From RFM01 command #8 CE8B (11. output and FIFO mode)
     // f3 f2 f1 f0 s1 s0 ff fe
     //  1  0  0  0  1  0  1  1
     // f: FIFO interrupt level = 8
     // s: FIFO fill start condition = reserved
     // ff=1: enable FIFO fill
     // fe=1: enable FIFO function

     // From RFM01 command #9 C081 (4. receiver setting command)
     // d1 d0 g1 g0 r2 r1 r0 en
     //  1  0  0  0  0  0  0  1
     // d: VDI source = clock recovery lock output
     // g: LNA gain = 0 dBm
     // r: DRSSI threshold = -103 dBm
     // en=1: enable receiver <--- only diff from command #7

     // From RFM01 command #10 C200 (7. Low Batt Detector & MCU Clock Div)
     // d2 d1 d0 t4 t3 t2 t1 t0
     //  0  0  0  0  0  0  0  0
     // d: frequency of CLK pin = 1MHz
     // t: low batt detection theshold = 2.2+0 V

     // From RFM01 command #11 A618 (3. frequency setting command)
     // Fc = 433.9MHz

     // From RFM01 command #12 CE89 (11. output and FIFO mode) (gangliontwitch has CE88)
     // f3 f2 f1 f0 s1 s0 ff fe
     //  1  0  0  0  1  0  0  1
     // f: FIFO interrupt level = 8
     // s: FIFO fill start condition = reserved
     // ff=0: disable FIFO fill
     // fe=1: enable FIFO function

     // From RFM01 command #14 CE8B (11. output and FIFO mode)
     // f3 f2 f1 f0 s1 s0 ff fe
     //  1  0  0  0  1  0  1  1
     // f: FIFO interrupt level = 8
     // s: FIFO fill start condition = reserved
     // ff=1: enable FIFO fill
     // fe=1: enable FIFO function

    // Reset Mode Command is not set so it defaults to DA00
    // (do not disable highly sensitive reset)

    /***************************
     * BEGIN RFM12b COMMANDS...
     ***************************/

    spi_select(true);
	rfm_transfer(0x0000);

    // 2. configuration setting command
    // 1 0 0 0 0 0 0 0 el ef b1 b0 x3 x2 x1 x0
    // el: enable TX register
    // ef: enable RX FIFO register
    // b: select band. 01 = 433MHz
    // x: load capacitor.
	//    0010 (0x2)=9.5pF (from CC RFM01)
	//    0111 (0x7)=12.0pF (from jeelib)
	//
    //                     ee
    //             10000000lfbbxxxx
    rfm_transfer(0b1000000001010111);

    // 3. Power Management Command
    // 1 0 0 0   0 0 1 0  er ebb et es ex eb ew dc
    // er : enable whole receiver chain (automatically turns on crystal,
    //      synth, baseband and RF front end)
    // ebb: enable RX baseband circuit (missing on RFM01)
    // et : enable TX (PLL & PA)
    // es : enable synthesiser (must be on to enable baseband circuits)
    // ex : enable crystal oscillator
    // eb : enable low batt detector
    // ew : enable wake-up timer
    // dc : disable clock output of CLK pin
    //                     eeeeeeed
    //                     rbtsxbwc
    rfm_transfer(0b1000001011011001);

    // 4. Frequency setting command
    // 1 0 1 0 F
    // F  = 1560 decimal = 0x618
    // Fc = 10 x 1 x (43 + F/4000) MHz = 433.9 MHz
    //
    rfm_transfer(0xA618); // 433.9MHz

    // 5. Data Rate command
    // 1 1 0 0   0 1 1 0   cs  r...
    // r  = 10 decimal
    // cs = 1
    // BitRate = 10000 / 29 / (R+1) / (1 + 7) = 3.918 kbps (from CCRFM01)
    rfm_transfer(0xC68A);


    // 6 Receiver control command
    // 1 0 0 1 0 P16 d1 d0 i2 i1 i0 g1 g0 r2 r1 r0
    //
    // p16: function of pin16. 1 = VDI output
    // d: VDI response time. 00=fast, 01=med, 10=slow, 11=always on
    // i: baseband bandwidth. 110=67kHz (CCRFM01=67kHz)
    // g: LNA gain. 00=0dB.
    // r: RSSI detector threshold. 000 = -103 dBm
    //
    //             10010Pddiiiggrrr
    rfm_transfer(0b1001010011000000);

    // 7. Digital filter command
    // 1 1 0 0 0 0 1 0 al ml 1 s 1 f2 f1 f0
    //
    // al: clock recovery (CR) auto lock control
    //     1=auto, 0=manual (set by ml).
    //     CCRFM01=0.
    // ml: enable clock recovery fast mode. CCRFM01=1
    // s :  data filter. 0=digital filter. (default & CCRFM01)=0
    // f : DQD threshold. CCRFM01=2; but RFM12b manual recommends >4
    //                     am
    //             11000010ll1s1fff
    rfm_transfer(0b1100001001101100);


    // 13 PLL setting
    // rfm_transfer(0xCC77);

    // 8. FIFO and Reset Mode Command
    // 1 1 0 0 1 0 1 0 f3 f2 f1 f0 sp al ff dr
    //
    // f: FIFO interrupt level = 8 (RFM01 & default)
    // sp: length of synchron pattern (not on RFM01!!!)
    // al: FIFO fill start condition. Default = sync-word.
    //     0=synchron pattern
    //     1=always fill
    // ff: enable FIFO fill
    // dr: disable hi sensitivity reset mode
    //
    //                         safd
    //             11001010ffffplfr
    rfm_transfer(0b1100101010001000);

    // 8. FIFO and Reset Mode Command
    // 1 1 0 0 1 0 1 0 f3 f2 f1 f0 sp al ff dr
    //
    // f: FIFO interrupt level = 8 (RFM01 & default)
    // sp: length of synchron pattern (not on RFM01!!!)
    // al: FIFO fill start condition. Default = sync-word.
    //     0=synchron pattern
    //     1=always fill
    // ff: enable FIFO fill
    // dr: disable hi sensitivity reset mode
    //
    //                         safd
    //             11001010ffffplfr
    rfm_transfer(0b1100101010001010);

    // 9 Synchron pattern command
    // rfm_transfer(0xCE55);


    // 11. AFC Command
    // 1 1 0 0 0 1 0 0 a1 a0 rl1 rl0 st fi oe en
    // 1 1 0 0 0 1 0 0  1  0   0   1  1  1  1  1
    // a: AFC auto-mode= keep offset when VDI hi (RFM01 and default)
    // rl: range limit= +15/-16 (433band: 2.5kHz)
    // st=1 st goes hi will store offset into output register
    // fi=1 Enable AFC hi accuracy mode (RFM01)
    // oe=1 Enable AFC output register
    // en=1 Enable AFC function
    rfm_transfer(0xC49F);

    // 12. TX config
    // rfm_transfer(0x9850); // !mp,9810=30kHz,MAX OUT

    // 15. wake-up timer command
    rfm_transfer(0xE000); // NOT USE

    // 16. Low duty cycle
    rfm_transfer(0xC80E); // NOT USE (last bit is 0 -> disable lower duty cycle mode)

    // 17. low battery detector and micro controller clock div
    rfm_transfer(0xC000); // 1.0MHz,2.2V
    spi_select(false);

    Serial.println("attaching interrupt");
    attachInterrupt(0, rfm12_interrupt, LOW);

    Serial.println("Done init.");
    return;
}

void setup() {
    Serial.begin(115200);
    Serial.print("\n[rf12serial2]\n");
    rf12_init_cc();
}

void loop() {

}


int main(void) {

  init();
  setup();

  while(true) {
    loop();
  }
}

