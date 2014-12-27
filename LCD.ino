
//  LCD  FUNCTIONS-- keep the ones you need. 

// clear the LCD
void clearLCD(){
 LCDSerial.write(0xFE);
 LCDSerial.write(0x01);

}

// move the cursor to a specific place
// e.g.: cursorSet(3,2) sets the cursor to x = 3 and y = 2
void cursorSet(int xpos, int ypos){  
 int count = 128;
 LCDSerial.write(254);           
  if(ypos==1) 
    count = 192;
   
 LCDSerial.write(count+xpos); //Row position 
} 

// turn on backlight
void backlightOn(int minutes){
    LCDSerial.write(0x7C);   //command flag for backlight stuff
    LCDSerial.write(157);    //light level.
   delay(10);  
//  LCDSerial.print(minutes); // use 0 minutes to turn the backlight on indefinitely   
}

// turn off backlight
void backlightOff(){
  //Serial.print(254, BYTE); 
  //Serial.print(70, BYTE);   
}




