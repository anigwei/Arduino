// Example 24.1
 
#include <LiquidCrystal.h> // we need this library for the LCD commands
 
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(4,5,6,7,8,9); // define our LCD and which pins to use
 
int a = 63;
int d = 3000; // used for display delayfloat
float b = 3.1415926;
 
void setup()
{
  lcd.begin(16, 2); // need to specify how many columns and rows are in the LCD unit
  lcd.clear();      // this clears the LCD. You can use this at any time
}
 
void loop()
{
  lcd.clear();
  lcd.setCursor(0,0);
  // positions starting point on LCD, column 0, row 0 (that is, the top left of our LCD)
  lcd.print("Hello!");
  // prints "Hello" at the LCD's cursor position defined above
  lcd.setCursor(0,1);
  // positions starting point on LCD, column 0, row 1 (that is, the bottom left of our LCD)
  lcd.println("This is fun     ");
  // note the rest of the line is padded out with spaces
  delay(d);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("a = ");
  lcd.print(a); // this will immediately follow "a = "
  lcd.setCursor(0,1);
  lcd.print("pi = ");
  lcd.print(b,7);
  // the 7 means 7 decimal places. You can also replace this with DEC, HEX, BIN as with
  // other *.print functions, as such:
  delay(d);
  lcd.clear();
  lcd.home(); // sets cursor back to position 0,0 - same as lcd.setCursor(0,0);
  lcd.print("a (HEX) = ");
  lcd.print(a, HEX); // this will immediately follow "a = "
  lcd.setCursor(0,1);
  lcd.print("a (BIN) = ");
  lcd.print(a,BIN);
  delay(d);
  lcd.noDisplay(); // turns off the display, leaving currently-displayed data as is
  delay(d);        // however this does not control the backlight
  lcd.display();   // resumes display
  delay(d);
}
