/* k-lfo v1.0
 *
 * Updates to KASTLE LFO code by J Tuffen for wonkystuff ltd. https://wonkystuff.co.uk
 * Original code by Vaclav Pelousek, Original header-comment included below
 *
 * - Removal of redundant code
 * - Reformatted (using VSCode auto-formatter)
 * - converted to PlatformIO project (was Arduino IDE based, now all settings are contained in platformio.ini)
 */

/*
KASTLE LFO v 1.0


Kastle Drum Features
  -8 drum synthesis styles
  -”noises” output for less tonal content
  -DRUM selects drum sounds
  -acceleration charge dynamic envelope
  -decay time
  -PITCH control with offset and CV input with attenuator
  -voltage-controllable clock with square and triangle output
  -stepped voltage generator with random, 8 step and 16 step loop mode
  -2 I/O CV ports that can be routed to any patch point
  -the main output can drive headphones
  -3x AA battery operation or USB power selectable by a switch
  -open source
  -durable black & gold PCB enclosure



  Writen by Vaclav Pelousek 2020
  based on the earlier kastle v1.5
  open source license: CC BY SA
  http://www.bastl-instruments.com

  -this is the code for the LFO chip of the Kastle
  -software written in Arduino 1.8.12 - used to flash ATTINY 85 running at 8mHz
  http://highlowtech.org/?p=1695
  -created with help of the heavenly powers of internet and several tutorials that you can google out
  -i hope somebody finds this code usefull (i know it is a mess :( )

  thanks to
  -Lennart Schierling for making some work on the register access
  -Uwe Schuller for explaining the capacitance of zener diodes
  -Peter Edwards for making the inspireing bitRanger
  -Ondrej Merta for being the best boss
  -and the whole bastl crew that made this project possible
  -Arduino community and the forums for picking ups bits of code to setup the timers & interrupts
  -v1.5 and kastle drum uses bits of code from miniMO DCO http://www.minimosynth.com/
 */

#include <Arduino.h>

uint8_t analogChannelRead = 1;
uint16_t analogValues[3];
uint16_t lastAnalogValues[3];
uint8_t runglerByte;
uint16_t upIncrement = 0;
uint16_t downIncrement = 255;
uint8_t waveshape;

uint16_t runglerOut;
bool lastDoReset;
const uint8_t runglerMap[8] = {
    0, 80, 120, 150, 180, 200, 220, 255};

int out, in;
const bool usePin[4] = {
    true, false, true, false};
uint8_t lfoValue = 0;
bool lfoFlop = true;
bool doReset = false;

uint16_t wsMap[10] = {
    0, 50, 127, 191, 255, 80, 157, 180, 220, 254};

#define WSMAP_POINTS 5

uint8_t mapLookup[256];

uint32_t curveMap(uint8_t value, uint8_t numberOfPoints, uint16_t *tableMap)
{
    uint32_t inMin = 0, inMax = 255, outMin = 0, outMax = 255;
    for (int i = 0; i < numberOfPoints - 1; i++)
    {
        if (value >= tableMap[i] && value <= tableMap[i + 1])
        {
            inMax = tableMap[i + 1];
            inMin = tableMap[i];
            outMax = tableMap[numberOfPoints + i + 1];
            outMin = tableMap[numberOfPoints + i];
            i = numberOfPoints + 10;
        }
    }
    return map(value, inMin, inMax, outMin, outMax);
}

void createLookup(void)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        mapLookup[i] = curveMap(i, WSMAP_POINTS, wsMap);
    }
}

// #### FUNCTIONS TO ACCES ADC REGISTERS
void init(void)
{
    ADMUX = 0;
    // prescaler = highest division
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADIE);
    sei();
}

void startConversion(void)
{
    ADCSRA |= _BV(ADSC); // start conversion
}

bool isConversionFinished(void)
{
    return (ADCSRA & (1 << ADIF));
}

bool isConversionRunning(void)
{
    return !isConversionFinished();
}

uint16_t getConversionResult(void)
{
    uint16_t result = ADCL;
    return result | (ADCH << 8);
}

// channel 8 can be used to measure the temperature of the chip
void connectChannel(const uint8_t number)
{
    ADMUX = number & 0x03; // limit to lower 2 bits - upper bits are always zero
}

void setTimers(void)
{
    TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
    TCCR0B = 0 << WGM02 | 1 << CS00;

    //  setup timer 0 to run fast for audiorate interrupt
    TCCR1 = 0;         // stop the timer
    TCNT1 = 0;         // zero the timer
    GTCCR = _BV(PSR1); // reset the prescaler
    OCR1A = 250;       // set the compare value

    TIMSK = _BV(OCIE1A); // interrupt on Compare Match A
    // start timer, ctc mode, prescaler clk/1
    TCCR1 = _BV(CTC1) | _BV(CS12); //_BV(CS10)  | _BV(CS11) ;// //| _BV(CS10); //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
    bitWrite(TCCR1, CS12, 0);
    sei();
}

void setFrequency(int _freq)
{
    _freq = (2048 - (_freq << 3)) + 20;
    uint8_t preScaler = _freq >> 7;
    preScaler += 1; //*2
    for (uint8_t i = 0; i < 4; i++)
        bitWrite(TCCR1, CS10 + i, bitRead(preScaler, i));
    uint8_t compare = _freq;
    bitWrite(compare, 7, 0);
    OCR1A = compare + 128;
}

void setup(void)
{
    digitalWrite(5, HIGH);
    pinMode(4, INPUT);
    digitalWrite(4, HIGH);
    createLookup();
    setTimers(); // setup audiorate interrupt
    runglerByte = random(255);
    pinMode(0, OUTPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(1, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    digitalWrite(4, LOW);
    init();
    connectChannel(analogChannelRead);
    startConversion();
}

ISR(ADC_vect)
{
    lastAnalogValues[analogChannelRead] = analogValues[analogChannelRead];
    analogValues[analogChannelRead] = getConversionResult();
    if ((analogChannelRead == 2) &&
        (lastAnalogValues[2] != analogValues[2]))
    {
        setFrequency(mapLookup[analogValues[2] >> 2]);
    }

    analogChannelRead++;
    while (!usePin[analogChannelRead])
    {
        analogChannelRead++;
        if (analogChannelRead > 3)
        {
            analogChannelRead = 0;
        }
    }

    if (analogChannelRead > 2)
    {
        analogChannelRead = 2;
    }
    connectChannel(analogChannelRead);

    startConversion();
}

uint32_t resetHappened = 0;
bool makeReset = false;

void renderRungler(void)
{
    uint8_t analogueByte = analogValues[0] >> 2;      // convert 10-bit ADC value to 8-bit
    uint8_t newBit = bitRead(runglerByte, 7) ? 1 : 0; // remember the MSbit as a '1' or '0'
    runglerByte <<= 1;                                // shift rungler byte left one place - **LSBit is now zero**
    if (analogueByte > 162)
    {
        newBit = TCNT0 >> 7; // randomise newBit
    }
    else if (analogueByte >= 150) // if 'bit' input is in the middle (less than 162, but bigger than 149)
    {
        newBit = !newBit; // invert newBit - rungler sequence will be 16 steps
    }
    runglerByte |= newBit; // set the LSBit

    // extract bits 0,3, and 5 to form the rungler's output
    runglerOut = runglerByte & 0x01;                                    // initialise runglerOut with bit zero of rungler byte - bitWrite(runglerOut, 0, bitRead(runglerByte, 0));
    runglerOut = runglerByte & _BV(3) ? runglerOut | 0x02 : runglerOut; // bit 1 of runglerOut is set by bit 3 of runglerByte - bitWrite(runglerOut, 1, bitRead(runglerByte, 3));
    runglerOut = runglerByte & _BV(5) ? runglerOut | 0x04 : runglerOut; // bit 2 of runglerOut is set by bit 5 of runglerByte - bitWrite(runglerOut, 2, bitRead(runglerByte, 5));
    runglerOut = runglerMap[runglerOut];
}

bool lastSquareState, squareState;
void loop(void)
{
    // pure nothingness
    doReset = bitRead(PINB, PINB3);
    if (!lastDoReset && doReset)
    {
        resetHappened++;
        if (resetHappened > 8)
            makeReset = false;
        if (makeReset)
            lfoValue = 0, lfoFlop = 0;
        renderRungler();
    }
    lastDoReset = doReset;

    lastSquareState = squareState;
    if (lfoValue < 128)
        squareState = 1;
    else
        squareState = 0;

    if (lastSquareState != squareState)
    {
        if (makeReset)
            renderRungler();
        bitWrite(PORTB, PINB2, squareState);
    }

    OCR0B = constrain(out, 0, 255);
    OCR0A = runglerOut;
}

ISR(TIMER1_COMPA_vect) // audiorate interrupt
{
    lfoValue++;

    if (lfoValue == 0)
    {

        lfoFlop = !lfoFlop;
        if (lfoFlop)
        {
            if (resetHappened > 4)
                makeReset = false, resetHappened = 0;
            else
                makeReset = true, resetHappened = 0;
        }
    }
    if (lfoFlop)
        out = 255 - lfoValue;
    else
        out = lfoValue;

    TCNT1 = 0;
}
