/* 
 by Manny Q.7-9-22
 
 Terms of use
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/
// included libraries:
#include <LiquidCrystal.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN
// variables
boolean ready = 0;
const byte address[6] = "00001";
int buttonValue = 0,batteryValue = 0,butDown = 0, butUp = 0,butLeft = 0,butRight = 0,butSelect = 0,currentPos = 0, menuMax = 0, currentMenu = 0, drawn, safe = 0;
String rItems[] = {"SAT:","LON:","LAT:","STATUS:","BAT:     MPU ACC","MODE:","BAR:"};
String infoText[] = {"By Manny Q.","Stuff Hacked Here", "July-2022"};
String mainMenu[] = {".Transmit",".Receive", ".Info", ".Battery"};
String subMenu[] = {" ","RTH", "Land"};
unsigned long timer = (millis() / 10);
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
  // set up the LCD's number of columns and rows:
  lcd.begin(20,4);
  pinMode(A5, INPUT);
  pinMode(A3, INPUT);
  intro();
  delay(2000);
  drawn = 0;
  currentMenu = 0;
}

void loop() {
  buttonValue = analogRead(A5);
  batteryValue = analogRead(A3);
  ready = radio.available();
  drawCursor();
  getButtons();
  menuMax = getMenu();
  controlMenu();
  drawMenu();
  if(currentMenu == 11)
  {
    lcd.setCursor(7,1);
    lcd.print(batteryValue/74.925);
    delay(250);
  }
  if(currentMenu == 5)
  {
    char text[5] = "";                 //Saving the incoming data
    radio.read(&text, sizeof(text));    //Reading the data
    lcd.setCursor(5,0);
    lcd.print(text);
    delay(5);
  }
  /*lcd.setCursor(10,1);
  lcd.print(buttonValue);*/
}

void intro()
{
  // set the cursor to column 0, line 1(note: line 1 is the second row, since counting begins with 0):
  // draw welcome message
  lcd.setCursor(0, 0);
  lcd.print("********************");
  lcd.setCursor(0, 1);
  lcd.print("* MANNYS' RECEIVER *");
  lcd.setCursor(0, 2);
  lcd.print("*  CON: JULY-2022  *");
  lcd.setCursor(0, 3);
  lcd.print("********************");
}

void drawCursor()
{
  lcd.setCursor(0,currentPos);
  lcd.print(">");
  if(currentPos >= 0 && currentPos <=menuMax )
  {
    lcd.setCursor(0,currentPos-1);
    lcd.print(" ");
    lcd.setCursor(0,currentPos+1);
    lcd.print(" ");
  }
}

void drawMenu () 
{
  //delay(1);
  if(currentMenu == 0 && drawn == 0)
  {
    lcd.clear();
    drawn = 1;
    for(int i = 0; i< 4; i++)
    {
      lcd.setCursor(1,i);
      lcd.print(mainMenu[i]);
    }
  }
  if(currentMenu == 1 && drawn == 0)
  {
     lcd.clear();
     drawn = 1;
     for(int i = 0; i< 3; i++)
     {
       lcd.setCursor(1,i);
       lcd.print(subMenu[i]);
     }
  }
  if(currentMenu == 2 && drawn == 0)
  {
    drawn = 1;
    returnToHome();
  }
  if(currentMenu == 3 && drawn == 0)
  {
    drawn = 1;
    land();
  }
  if(currentMenu == 4 && drawn == 0)
  {
    drawn = 1;
    lcd.clear();
    lcd.setCursor(3,1);
    lcd.print("Receiving Data");
    lcd.setCursor(3,2);
    for(int i = 0; i<14; i++)
    {
      lcd.print(".");
      delay(175);
    }
    if (ready)
    {
      currentMenu = 5;
      drawn = 0;
    }
    else
    {
      lcd.clear();
      lcd.setCursor(2,1);
      lcd.print("No Data Received");
      lcd.setCursor(6,2);
      lcd.print("Exiting");
      delay(2000);
      currentMenu = 0;
      drawn = 0;
    }
  }
  if(currentMenu == 5 && drawn == 0)//change to connection established
  {
    // List 1st 4 options linearly
    lcd.clear();
    for(int i = 0; i<4; i++)
    {
      lcd.setCursor(1,i);
      lcd.print(rItems[i]);
    }
    drawn = 1;
  }
  if(currentMenu == 6 && drawn == 0)//change to connection established
  {
    // List 1st 4 options linearly
    lcd.clear();
    for(int i = 4; i<8; i++)
    {
      lcd.setCursor(1,i-4);
      lcd.print(rItems[i]);
    }
    drawn = 1;
  }
  if(currentMenu == 7 && drawn == 0)
  {
    drawn = 1;
    lcd.clear();
    lcd.setCursor(0,0);
    for(int i = 0; i< 3; i++)
    {
      lcd.setCursor(1,i);
      lcd.print(infoText[i]);
    }
  }
  if(currentMenu == 11 && drawn == 0)
  {
    drawn = 1;
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("Battery Voltage");
    lcd.setCursor(12,1);
    lcd.print("V");
   if(batteryValue<786)
    {
      lcd.setCursor(2,2);
      lcd.print("Status Low   "); 
    }
   if(batteryValue>921)
    {
      lcd.setCursor(2,2);
      lcd.print("Status High  "); 
    }
    else
    {
      lcd.setCursor(2,2);
      lcd.print("Status: Normal"); 
    }
    
  }
}


byte getMenu()
{
  int x = 0;
  if (currentMenu == 0)
  {
    x = (sizeof(mainMenu)/6);
  }
  if(currentMenu == 1)
  {
    x = (sizeof(subMenu)/6);
  }
  return x;
}

void getButtons()
{
  // Change the button status based on buttonValue
  switch (buttonValue) 
  {
    case 92:    // Left button Pressed
      butLeft = 1;
      break;
    case 179:    // Down Button Pressed
      butDown = 1;
      break;
    case 326:    // Select Button Pressed
      butSelect = 1;
      break;
    case 510:    // Right button pressed
      butRight = 1;
      break;
    case 697:    // Up button pressed
      butUp = 1;
      break;
     default:
     butLeft = 0;
     butDown = 0;
     butSelect = 0;
     butRight = 0;
     butUp = 0;
      break;
  }
}

void controlMenu()
{
  if(currentMenu == 0)
  {
    if(butDown == 1 && currentPos <= menuMax-2)
    {
      currentPos += 1;
      delay(175);
    }
    if(butUp == 1 && currentPos >0 )
    {
      currentPos -= 1;
      delay(175);
    }
    if(butSelect == 1 && currentPos == 0)
    {
      currentMenu = 1;
      safe = 0;
      drawn = 0;
    }
    if(butSelect == 1 && currentPos == 1)
    {
      currentMenu = 4;
      drawn = 0;
    }
    if(butSelect == 1 && currentPos == 2)
    {
      currentMenu = 7;
      drawn = 0;
    }
    if(butSelect == 1 && currentPos == 3)
    {
      currentMenu = 11;
      drawn = 0;
    }
  }

  if(currentMenu == 1)
  {
  
    if(butDown == 1 && currentPos <= menuMax-2)
    {
      currentPos += 1;
      delay(175);
    }
    if(butUp == 1 && currentPos >0 )
    {
      currentPos -= 1;
      delay(175);
    }
    if(butSelect == 1 && currentPos == 1)
    {
      currentMenu = 2;
      drawn = 0;
    }
    if(butSelect == 1 && currentPos == 2)
    {
      currentMenu = 3;
      drawn = 0;
    }
  }
  
  if (currentMenu != 0)
  {
    if (butLeft == 1 && currentMenu >0 && currentMenu != 6)
    {
       currentPos = 0;
       currentMenu = 0;
       drawn = 0;
    }
    if (butRight == 1 && currentMenu == 5)
    {
       currentMenu = 6;
       drawn = 0;
    }
    if (butLeft == 1 && currentMenu == 6)
    {
       currentMenu = 5;
       drawn = 0;
    }     
  }
}

void returnToHome()
{
  lcd.clear();
  lcd.setCursor(1,1);
  lcd.print("Returning to home");
}
void land()
{
  lcd.clear();
  lcd.setCursor(1,1);
  lcd.print("Landing");
}
