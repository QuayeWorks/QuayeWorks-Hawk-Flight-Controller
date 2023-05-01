//Channel 1 = analog input 0
//Channel 2 = analog input 1
//Channel 3 = analog input 2
//Channel 4 = analog input 3
//Channel 5 = analog input 4
//Channel 6 = analog input 5
//PPM output = output 2

int delay_counter;

void setup(){
  TIMSK0 &= ~_BV(TOIE0);          //Disable timer0 as it will consume a lot of time.
  pinMode(3, OUTPUT);             //Pin 3 will be the PPM output.
  while(PINC & B00000001);        //While analog input 0 is low.
}

void loop(){
  //Channel 1
  while(!(PINC & B00000001));     //While analog input 0 is low.
  PORTD |= B00001000;             //Set output 2 high.
  delayMicroseconds(100);         //Delay for 100 us.
  PORTD &= B11110111;             //Set output 3 low.
  //Channel 2
  while(!(PINC & B00000010));     //While analog input 1 is low.
  PORTD |= B00001000;             //Set output 3 high.
  delayMicroseconds(100);         //Delay for 100 us.
  PORTD &= B11110111;             //Set output 3 low.
  //Channel 3
  while(!(PINC & B00000100));     //While analog input 2 is low.
  PORTD |= B00001000;             //Set output 3 high.
  delayMicroseconds(100);         //Delay for 100 us.
  PORTD &= B11110111;             //Set output 3 low.
  //Channel 4
  while(!(PINC & B00001000));     //While analog input 3 is low.
  PORTD |= B00001000;             //Set output 3 high.
  delayMicroseconds(100);         //Delay for 100 us.
  PORTD &= B11110111;             //Set output 3 low.
  //Channel 5
  while(!(PINC & B00010000));     //While analog input 4 is low.
  PORTD |= B00001000;             //Set output 3 high.
  delayMicroseconds(100);         //Delay for 100 us.
  PORTD &= B11110111;             //Set output 3 low.
  //Channel 6
  while(!(PINC & B00100000));     //While analog input 5 is low.
  PORTD |= B00001000;             //Set output 3 high.
  delayMicroseconds(100);         //Delay for 100 us.
  PORTD &= B11110111;             //Set output 3 low.
  //Stop pulse
  while(PINC & B00100000);        //While analog input 5 is high.
  PORTD |= B00001000;             //Set output 3 high.
  delayMicroseconds(100);         //Delay for 100 us.
  PORTD &= B11110111;             //Set output 3 low.
}
