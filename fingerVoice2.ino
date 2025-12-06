/****************************************************************************
  Microduino source code for the PAPER
  "A Syllable Based Input System Via Finger Touching"
  Wenjie Chen, Yangyang Ma
  @ECNU
*****************************************************************************/


// 左手端向逐个手指写高电平，检测右手端有无被置高，
// 如有，说明左右手互联，然后检测左手有没有其他手指相连，
//       过一段时间再检查有无抖动，如有抖动状态重置，否则(超过50ms无抖动)认为左右手稳定接触，产生输出
// 如无，说明右手没有接触左手，或者该左手指未参与连接，换下一左手指重复。


#include<I2Cdev.h>
#include<Wire.h>
// #include"U8glib.h"
#include"math.h"
#include<MPU6050.h>
// #include<MsTimer2.h>


//  #if 1
//      #define ERROR_REPORT(s)	Serial.println(s);
//  #else
//   	#define ERROR_REPORT(s) 
//  #endif


// ERROR_REPORT("wrong argument: x= [%d]", 10);  ====>   "<ERROR> wrong argument: x= [10]\n"
#if 1
	char report_str[256];
	#define ERROR_REPORT(format, ...) \
		do { sprintf(report_str, "<ERROR> " format, ##__VA_ARGS__); \
			 Serial.println(report_str); } while (0);
#else
	#define ERROR_REPORT(format, ...)
#endif


MPU6050 accelgyro;

float 		AX_old,AY_old,AZ_old;



//typedef unsigned char byte;

int left_fingers_touched  = 0;			// 低五位表示五指，大指为第0位。每位触碰为1，未触碰为0.
int right_fingers_touched = 0;
int last_left_fingers_touched  = 0;		// for de-bounce
int last_right_fingers_touched = 0;
int finger_tone = 0;

int fourtone = 0;

int syllable_code = 0;       // |----- not used -------|3bit fourtone | 5bit left (consonant) | 5bit right (vowel) |
char syllable_str[8];        // "zhuang1"


/* ********************************************************************
fourtone:
   1：-  阴平；2：/  阳平； 3：v 上声  4：\ 去声;     5: 轻声；  

left fingers (consonants)：
    ==============================================
    No. consonant  fingers    bitcode      dec.
    ----------------------------------------------
     1    b        1          0b_00001     1
     2    p        2          0b_00010     2
     3    m        3          0b_00100     4
     4    f        4          0b_01000     8
     5    d        12         0b_00011     3
     6    t        13         0b_00101     5
     7    n        14         0b_01001     9
     8    l        15         0b_10001     17
     9    g        123        0b_00111     7
    10    k        124        0b_01011     11
    11    h        125        0b_10011     19
    12    j        23         0b_00110     6
    13    q        34         0b_01100     12
    14    x        45         0b_11000     24
    15    zh       12345      0b_11111     31
    16    ch       1345       0b_11101     29
    17    sh       2345       0b_11110     30
    18    r        5          0b_10000     16
    19    z        1234       0b_01111     15
    20    c        134        0b_01101     13
    21    s        234        0b_01110     14
    22    y        145        0b_11001     25
    23    w        135        0b_10101     21
    24   (zero)    345        0b_11100     28
    ==============================================

right fingers (vowels)：
    v= yu
    ===================================================
    No.     vowel   fingers same_as bitcode     dec.
    ---------------------------------------------------
    1       a       1               0b_00001    1
    2       o       2       uo      0b_00010    2
    3       e       3               0b_00100    4
    4       i       4       er      0b_01000    8
    5       u       5               0b_10000    16
    6       v       45      ui      0b_11000    24
    7       ai      14              0b_01001    9
    8       ei      34              0b_01100    12
    9       ui      45      v       0b_11000    24
    10      ao      12              0b_00011    3
    11      ou      25              0b_10010    18
    12      iu      245             0b_11010    26
    13      ie      134             0b_01101    13
    14      ue      345             0b_11100    28
    15      er      4       i       0b_01000    8
    16      an      15              0b_10001    17
    17      en      35              0b_10100    20
    18      in      125             0b_10011    19
    19      un      234     vn      0b_01110    14
    20      vn      234     un      0b_01110    14
    21      ang     2345            0b_11110    30
    22      eng     1245            0b_11011    27
    23      ing     1235            0b_10111    23
    24      ong     1345    iong    0b_11101    29
    25      ia      145     ua      0b_11001    25
    26      iao     124             0b_01011    11
    27      ian     123             0b_00111    7
    28      iang    12345   uang    0b_11111    31
    29      iong    1345    ong     0b_11101    29
    30      ua      145     ia      0b_11001    25
    31      uai     345     ue      0b_11100    28
    32      uan     135     van    0b_10101    21
    33      van     135     uan     0b_10101    21
    34      uang    12345   iang    0b_11111    31
    35      uo      2       o       0b_00010    2
    =====================================================
************************************************************************** */

unsigned long lastDebounceTime = 0;  		// the last time while finger touchred status changed.
const unsigned long DEBOUNCE_DELAY = 50;    // the debounce time; increase if the output flickers


const int LEFT_FINGER_PIN[5]  = {2,3,4,5,6};	//D2-PIN: 左大拇指, ...
const int RIGHT_FINGER_PIN[5] = {7,8,9,10,11};	//D7-PIN: 右大拇指, ...



void setHighOut(int finger_index)  //左手第i指输出高电平（其它指输入）,称为active-finger
{
    static int s_active_finger = -1;	//static, record last active-finger index, initialize to -1

    // restore pin mode for last active finger pin
    //if(s_active_finger >= 0 and s_active_finger < 4 )
    if(s_active_finger >= 0 && s_active_finger <= 4 )
    {
        digitalWrite(LEFT_FINGER_PIN[s_active_finger], LOW);
        pinMode(LEFT_FINGER_PIN[s_active_finger], INPUT);
    }

    s_active_finger = finger_index;

    pinMode(LEFT_FINGER_PIN[finger_index], OUTPUT);
    digitalWrite(LEFT_FINGER_PIN[finger_index], HIGH);
}


int check_right_fingers(void)
{
    int ret = 0;

    for(int i=0; i<5; i++)
    {
        ret += (( digitalRead(RIGHT_FINGER_PIN[i]) == HIGH)? 1<<i : 0) ;
    }

    return ret;
}


int check_left_fingers(int active_finger)  // 0..4
{
    int ret = 1 << active_finger;

    for(int i=active_finger +1; i<5; i++)
    {
        ret += (( digitalRead(LEFT_FINGER_PIN[i]) == HIGH)? 1<<i : 0) ;
    }

    return ret;
}





void setup()
{

    for(int i=0; i<5; i++)
        pinMode(LEFT_FINGER_PIN[i], INPUT);

    for(int i=0; i<5; i++)
        pinMode(RIGHT_FINGER_PIN[i], INPUT);

 
    Serial.begin(9600);//115200 for computer;9600 for bluetooth 
    Serial.println("Ready?");
    
    Wire.begin();
    accelgyro.initialize();
 
    AX_old = AY_old = AZ_old = 0;
    
    Serial.println("Go!");
}


//@_@: 如果要低功耗运行，可以把下面的程序放在定时响应函数里。定时唤醒执行。

void loop()
{
    right_fingers_touched = 0;
    left_fingers_touched = 0;
    finger_tone = 0;

    for(int i=0; i<5; i++)  //left finger
    {
        setHighOut(i);

        if ((right_fingers_touched = check_right_fingers()) == 0)  // right fingers have not touched left out_finger
        {
            continue;
        }
        else
        {

            left_fingers_touched = check_left_fingers(i);

            //
            // de-bounce. *******************
            //
            if ( (left_fingers_touched != last_left_fingers_touched) || (right_fingers_touched != last_right_fingers_touched) )
            {
                lastDebounceTime = millis();
                last_left_fingers_touched = left_fingers_touched;
                last_right_fingers_touched = right_fingers_touched;

                break;
            }
            else
            {
                if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
                {
                    // left_fingers_touched, right_fingers_touched  are steable for use
                    //
                    // DO MAIN WORK HERE:
                    //
                    finger_tone = check_tone();
                    if (finger_tone == 0)
                    {
                        Serial.println("tone = 0");
                        break;
                    }

                    syllable_code = (finger_tone << 10) + (left_fingers_touched << 5) + right_fingers_touched;
                    //syllable_code = (check_tone() << 10) + (left_fingers_touched << 5) + right_fingers_touched;
                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    
                    get_syllable_str_from_code(syllable_str, syllable_code );

                    //Serial.println(syllable_code);
                    Serial.println(syllable_str);
                    
                    // 等待脱离接触，一直接触不会一直发声。
                    while ((right_fingers_touched = check_right_fingers()) != 0)
                    {
                        ;
                    }

                    //直到断开一次接触，重新开始准备下一轮（检测下一个音节）：
                    last_left_fingers_touched  = 0;
                    last_right_fingers_touched = 0;
                    lastDebounceTime = 0;
                }

                break;
            }

        }

    } //    for(int i=0; i<5; i++)  //left finger


}



//@_@: 要提高效率，可以预先全部展开，用最终的数字进行运算。
//		可以在头上用注释说明下思路，最后用化简后的公式实际运算。
int check_tone(void)
{

#define Gx_offset -2.1
#define Gy_offset 1.7
#define Gz_offset 2.5

   //float 		standard = 0.1;
    
    int16_t 	ax,ay,az;
    int16_t 	gx,gy,gz;//储存原始数据
    float 		Ax,Ay,Az;//加速度
    //float 		Gx,Gy,Gz;//角速度
    float 		Angel_X,Angel_Y,Angel_Z;//角度

   /* int i=0;
    int32_t axsum,aysum,azsum;
    axsum=aysum=azsum=0;

    for(i=0; i<10; i++)
    {
        accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
        axsum += ax;
        aysum += ay;
        azsum += az;
    }*/

    accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    Ax = ax/16384.00;		//@_@: 这个似乎没有必要，后面约掉了
    Ay = ay/16384.00;
    Az = az/16384.00;

    Angel_X = atan(Ax/sqrt(Az*Az+Ay*Ay))*180/3.14;		//@_@: 上下都/16384/i,  约掉了。用 axsum，aysum即可。
    Angel_Y = atan(Ay/sqrt(Az*Az+Ax*Ax))*180/3.14;
    Angel_Z = atan(Az/sqrt(Ax*Ax+Ay*Ay))*180/3.14;

    Angel_X += Gx_offset;
    Angel_Y += Gy_offset;
    Angel_Z += Gz_offset;

    if ( Angel_Y<15 && Angel_Y>-15 && Angel_X<-30 )//Angel_X<-70 && Angel_X>-90 && Angel_Y<15 && Angel_Y>-15
    {
        return 1;
    }
    else if ( Angel_Y<-15 && Angel_Y>-90 )//Angel_X<-20 && Angel_X>-70 && Angel_Y<-15 && Angel_Y>-60
    {
        return 2;
    }
    else if ( Angel_Z<-45 && Angel_Z>-90 )//Angel_X<20 && Angel_X>-20 && Angel_Z<-45 && Angel_Z>-90
    {
        return 3;
    }
    else if ( Angel_Y<70 && Angel_Y>15 && Angel_X<-20)//Angel_X<-30 && Angel_X>-65 && Angel_Y<80 && Angel_Y>15
    {
        return 4;
    }
    else if ( Angel_Y>45 && Angel_X>-20 )//Angel_Y>45 && Angel_X>-20 && Angel_X<20 && Angel_Z<20 && Angel_Z>-20
    {
        return 5;
    }
    else
    {
        return 0;
    }

    /*if(Angel_X-AX_old>standard||AX_old-Angel_X>standard)		//@_@: 这段没看明白：AX_old 初始为0， standard 恒为0.1, 只要Angel_accX > 0.1 || Angel_accX < -0.1 即可？
    {
        AX_old=Angel_X;
    }
    if(Angel_Y-AY_old>standard||AY_old-Angel_Y>standard)
    {
        AY_old=Angel_Y;
    }
    if(Angel_Z-AZ_old>standard||AZ_old-Angel_Z>standard)
    {
        AZ_old=Angel_Z;
    }
    if(AY_old > 45)
    {     
    	 return 1;
    }  
    if(AZ_old > 45)
    {  
    	 return 2;
    }   
    if(AY_old < -45)
    {  
    	 return 3;
    }  
    if (AX_old < -45)
    {
        return 4;
    }   
    return 5;
    */

}


// syllable_str: INOUT. length should >=8
// syllable_code: IN. 
void get_syllable_str_from_code(char * syllable_str, int syllable_code)
{


    int consonant_code = 0;
    int vowel_code = 0;
    int tone_code  = 0;

    
    consonant_code = (syllable_code & 0x3E0 ) >> 5;     // 0x3E0 == 0b11'1110'0000 
    vowel_code     =  syllable_code & 0x1F;             // 0x1F == 0b11111
    tone_code      = (syllable_code & 0x1C00 ) >> 10;     // 0x3E0 == 0b1'1100'0000'0000    

    char *p_vowel = syllable_str;
    char *p_tone  = p_vowel;

    // ** (1) consonant ***********************************

    switch (consonant_code)
    {
        case    1:  strcpy(syllable_str, "b");    p_vowel++;     break;
        case    2:  strcpy(syllable_str, "p");    p_vowel++;     break;
        case    4:  strcpy(syllable_str, "m");    p_vowel++;     break;
        case    8:  strcpy(syllable_str, "f");    p_vowel++;     break;
        case    3:  strcpy(syllable_str, "d");    p_vowel++;     break;
        case    5:  strcpy(syllable_str, "t");    p_vowel++;     break;
        case    9:  strcpy(syllable_str, "n");    p_vowel++;     break;
        case    17: strcpy(syllable_str, "l");    p_vowel++;     break;
        case    7:  strcpy(syllable_str, "g");    p_vowel++;     break;
        case    11: strcpy(syllable_str, "k");    p_vowel++;     break;
        case    19: strcpy(syllable_str, "h");    p_vowel++;     break;
        case    6:  strcpy(syllable_str, "j");    p_vowel++;     break;
        case    12: strcpy(syllable_str, "q");    p_vowel++;     break;
        case    24: strcpy(syllable_str, "x");    p_vowel++;     break;
        case    31: strcpy(syllable_str, "zh");   p_vowel+=2;     break;
        case    29: strcpy(syllable_str, "ch");   p_vowel+=2;     break;
        case    30: strcpy(syllable_str, "sh");   p_vowel+=2;     break;
        case    16: strcpy(syllable_str, "r");    p_vowel++;     break;
        case    15: strcpy(syllable_str, "z");    p_vowel++;     break;
        case    13: strcpy(syllable_str, "c");    p_vowel++;     break;
        case    14: strcpy(syllable_str, "s");    p_vowel++;     break;
        case    25: strcpy(syllable_str, "y");    p_vowel++;     break;
        case    21: strcpy(syllable_str, "w");    p_vowel++;     break;
        case    28: strcpy(syllable_str, "");                    break;

        default: /*ERROR*/
			ERROR_REPORT("Wrong Left fingers combination:[%d (0x%x)]", consonant_code,  consonant_code);
    } //   switch (consonant_code)


    // ** (2) vowel ***********************************

    p_tone  = p_vowel;

    switch (vowel_code)
    {

        case    1 :     strcpy(p_vowel, "a"   );    p_tone ++;      break;               

        case    2 : /* o, uo  */
                    {   
                        switch (consonant_code) 
                        {   /*b, p, m, f, y, w, 0*/
                            case 1: case 2: case 4: case 8: case 25: case 21: case 28:
                                strcpy(p_vowel, "o"   );    p_tone ++;      break;
                        
                            default: 
                                strcpy(p_vowel, "uo"   );   p_tone += 2;    break;
                        }
                        break;
                     }

        case    4 :     strcpy(p_vowel, "e"   );    p_tone ++;      break;               

        case    8 : /* er, i */
                    {   
                        switch (consonant_code) 
                        {   /*0*/
                            case 28:
                                strcpy(p_vowel, "er"   );   p_tone += 2;    break;
                        
                            default: 
                                strcpy(p_vowel, "i"   );    p_tone ++;      break;
                        }
                        break;
                     }
             
        case    16:     strcpy(p_vowel, "u"   );    p_tone ++;      break;               

        case    24: /* v, ui */
                    {   
                        switch (consonant_code) 
                        {   /*n, l, j, q, x, y*/
                            case 9: case 17: case 6: case 12: case 24: case 25:
                                strcpy(p_vowel, "v"   );    p_tone ++;;  break;
                        
                            default: 
                                strcpy(p_vowel, "ui"   );   p_tone += 2; break;
                        }
                        break;
                     }

        case    9 :     strcpy(p_vowel, "ai"  );    p_tone += 2;    break;               
        case    12:     strcpy(p_vowel, "ei"  );    p_tone += 2;    break;                    
        case    3 :     strcpy(p_vowel, "ao"  );    p_tone += 2;    break;               
        case    18:     strcpy(p_vowel, "ou"  );    p_tone += 2;    break;               
        case    26:     strcpy(p_vowel, "iu"  );    p_tone += 2;    break;               
        case    13:     strcpy(p_vowel, "ie"  );    p_tone += 2;    break;               

        case    28: /* ve, uai */
                    {   
                        switch (consonant_code) 
                        {   /*n, l, j, q, x, y*/
                            case 9: case 17: case 6: case 12: case 24: case 25:
                                strcpy(p_vowel, "ve"   );   p_tone += 2;    break;
                        
                            default: 
                                strcpy(p_vowel, "uai"   );  p_tone += 3;    break;
                        }
                        break;
                     }

        case    17:     strcpy(p_vowel, "an"  );    p_tone += 2;    break;               
        case    20:     strcpy(p_vowel, "en"  );    p_tone += 2;    break;               
        case    19:     strcpy(p_vowel, "in"  );    p_tone += 2;    break;               

        case    14: /* un, vn */       
                    {   
                        switch (consonant_code) 
                        {   /*j, q, x, y*/
                            case 6: case 12: case 24: case 25:
                                strcpy(p_vowel, "vn"   );   p_tone += 2;    break;
                        
                            default: 
                                strcpy(p_vowel, "un"   );   p_tone += 2;    break;
                        }
                        break;
                     }

        case    30:     strcpy(p_vowel, "ang" );    p_tone += 3;    break;               
        case    27:     strcpy(p_vowel, "eng" );    p_tone += 3;    break;               
        case    23:     strcpy(p_vowel, "ing" );    p_tone += 3;    break;               

        case    29: /* ong, iong */     
                    {   
                        switch (consonant_code) 
                        {   /*j, q, x, y*/
                            case 6: case 12: case 24: case 25:
                                strcpy(p_vowel, "iong");    p_tone += 4;    break;
                        
                            default: 
                                strcpy(p_vowel, "ong");     p_tone += 3;    break;
                        }
                        break;
                     }

        case    25: /* ia, ua */    
                    {   
                        switch (consonant_code) 
                        {   /*j, q, x, y*/
                            case 6: case 12: case 24: case 25:
                                strcpy(p_vowel, "ia");  p_tone += 2;    break;
                        
                            default: 
                                strcpy(p_vowel, "ua");  p_tone += 2;    break;
                        }
                        break;
                     }


        case    11:     strcpy(p_vowel, "iao" );    p_tone += 3;    break;               
        case    7 :     strcpy(p_vowel, "ian" );    p_tone += 3;    break;               

        case    31: /* iang, uang */      
                    {   
                        switch (consonant_code) 
                        {   /*n, l, j, q, x, y*/
                            case 9: case 17: case 6: case 12: case 24: case 25:
                                strcpy(p_vowel, "iang");    p_tone += 4;    break;
                        
                            default: 
                                strcpy(p_vowel, "uang");    p_tone += 4;    break;
                        }
                        break;
                     }
                  
        case    21: /* uan, van */     
                    {   
                        switch (consonant_code) 
                        {   /*j, q, x, y*/
                            case 6: case 12: case 24: case 25:
                                strcpy(p_vowel, "van"); p_tone += 3;    break;
                        
                            default: 
                                strcpy(p_vowel, "uan"); p_tone += 3;    break;
                        }
                        break;
                     }
  
        default: 	/*ERROR*/
			ERROR_REPORT("Wrong RIGHT fingers combination:[%d (0x%x)]", vowel_code,  vowel_code);
 
    }   //switch (vowel_code)


    // ** (3) fourtone ***********************************

    *p_tone = tone_code + '0';

    *(p_tone+1) = '\0';

    return;
}
