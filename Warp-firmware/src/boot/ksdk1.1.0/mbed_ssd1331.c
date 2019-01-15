
//this file was taken from os.mbed.com, full link: https://os.mbed.com/users/star297/code/ssd1331/file/4385fd242db0/ssd1331.cpp/


#include "mbed_ssd1331.h"
#include "devSSD1331.h"
#include <stdint.h>
#include "SEGGER_RTT.h"

#define countof(x) ( sizeof(x) / sizeof(x[0]) )
static const char font6x8[0x60][6] = {
    { 0x00,0x00,0x00,0x00,0x00,0x00 } , /*SPC */
    { 0x00,0x00,0x5F,0x00,0x00,0x00 } , /* !  */
    { 0x04,0x03,0x04,0x03,0x00,0x00 } , /* "  */
    { 0x28,0x7E,0x14,0x3F,0x0A,0x00 } , /* #  */
    { 0x24,0x2A,0x7F,0x2A,0x12,0x00 } , /* $  */
    { 0x23,0x13,0x08,0x64,0x62,0x00 } , /* %  */
    { 0x30,0x4E,0x59,0x26,0x50,0x00 } , /* &  */
    { 0x00,0x00,0x02,0x01,0x00,0x00 } , /* '  */
    { 0x00,0x00,0x1C,0x22,0x41,0x00 } , /* (  */
    { 0x41,0x22,0x1C,0x00,0x00,0x00 } , /* )  */
    { 0x22,0x14,0x08,0x14,0x22,0x00 } , /* *  */
    { 0x08,0x08,0x3E,0x08,0x08,0x00 } , /* +  */
    { 0x50,0x30,0x00,0x00,0x00,0x00 } , /* ,  */
    { 0x08,0x08,0x08,0x08,0x08,0x00 } , /* -  */
    { 0x60,0x60,0x00,0x00,0x00,0x00 } , /* .  */
    { 0x20,0x10,0x08,0x04,0x02,0x00 } , /* /  */
    { 0x3E,0x51,0x49,0x45,0x3E,0x00 } , /* 0  */
    { 0x00,0x42,0x7F,0x40,0x00,0x00 } , /* 1  */
    { 0x62,0x51,0x49,0x49,0x46,0x00 } , /* 2  */
    { 0x22,0x41,0x49,0x49,0x36,0x00 } , /* 3  */
    { 0x18,0x14,0x12,0x7F,0x10,0x00 } , /* 4  */
    { 0x2F,0x45,0x45,0x45,0x39,0x00 } , /* 5  */
    { 0x3E,0x49,0x49,0x49,0x32,0x00 } , /* 6  */
    { 0x01,0x61,0x19,0x05,0x03,0x00 } , /* 7  */
    { 0x36,0x49,0x49,0x49,0x36,0x00 } , /* 8  */
    { 0x26,0x49,0x49,0x49,0x3E,0x00 } , /* 9  */
    { 0x00,0x36,0x36,0x00,0x00,0x00 } , /* :  */
    { 0x00,0x56,0x36,0x00,0x00,0x00 } , /* ;  */
    { 0x00,0x08,0x14,0x22,0x41,0x00 } , /* <  */
    { 0x14,0x14,0x14,0x14,0x14,0x00 } , /* =  */
    { 0x41,0x22,0x14,0x08,0x00,0x00 } , /* >  */
    { 0x02,0x01,0x59,0x09,0x06,0x00 } , /* ?  */
    { 0x3E,0x41,0x5D,0x55,0x2E,0x00 } , /* @  */
    { 0x60,0x1C,0x13,0x1C,0x60,0x00 } , /* A  */
    { 0x7F,0x49,0x49,0x49,0x36,0x00 } , /* B  */
    { 0x3E,0x41,0x41,0x41,0x22,0x00 } , /* C  */
    { 0x7F,0x41,0x41,0x22,0x1C,0x00 } , /* D  */
    { 0x7F,0x49,0x49,0x49,0x41,0x00 } , /* E  */
    { 0x7F,0x09,0x09,0x09,0x01,0x00 } , /* F  */
    { 0x1C,0x22,0x41,0x49,0x3A,0x00 } , /* G  */
    { 0x7F,0x08,0x08,0x08,0x7F,0x00 } , /* H  */
    { 0x00,0x41,0x7F,0x41,0x00,0x00 } , /* I  */
    { 0x20,0x40,0x40,0x40,0x3F,0x00 } , /* J  */
    { 0x7F,0x08,0x14,0x22,0x41,0x00 } , /* K  */
    { 0x7F,0x40,0x40,0x40,0x00,0x00 } , /* L  */
    { 0x7F,0x04,0x18,0x04,0x7F,0x00 } , /* M  */
    { 0x7F,0x04,0x08,0x10,0x7F,0x00 } , /* N  */
    { 0x3E,0x41,0x41,0x41,0x3E,0x00 } , /* O  */
    { 0x7F,0x09,0x09,0x09,0x06,0x00 } , /* P  */
    { 0x3E,0x41,0x51,0x21,0x5E,0x00 } , /* Q  */
    { 0x7F,0x09,0x19,0x29,0x46,0x00 } , /* R  */
    { 0x26,0x49,0x49,0x49,0x32,0x00 } , /* S  */
    { 0x01,0x01,0x7F,0x01,0x01,0x00 } , /* T  */
    { 0x3F,0x40,0x40,0x40,0x3F,0x00 } , /* U  */
    { 0x03,0x1C,0x60,0x1C,0x03,0x00 } , /* V  */
    { 0x0F,0x70,0x0F,0x70,0x0F,0x00 } , /* W  */
    { 0x41,0x36,0x08,0x36,0x41,0x00 } , /* X  */
    { 0x01,0x06,0x78,0x02,0x01,0x00 } , /* Y  */
    { 0x61,0x51,0x49,0x45,0x43,0x00 } , /* Z  */
    { 0x00,0x00,0x7F,0x41,0x41,0x00 } , /* [  */
    { 0x15,0x16,0x7C,0x16,0x11,0x00 } , /* \  */
    { 0x41,0x41,0x7F,0x00,0x00,0x00 } , /* ]  */
    { 0x00,0x02,0x01,0x02,0x00,0x00 } , /* ^  */
    { 0x40,0x40,0x40,0x40,0x40,0x00 } , /* _  */
    { 0x00,0x01,0x02,0x00,0x00,0x00 } , /* `  */
    { 0x00,0x20,0x54,0x54,0x78,0x00 } , /* a  */
    { 0x00,0x7F,0x44,0x44,0x38,0x00 } , /* b  */
    { 0x00,0x38,0x44,0x44,0x28,0x00 } , /* c  */
    { 0x00,0x38,0x44,0x44,0x7F,0x00 } , /* d  */
    { 0x00,0x38,0x54,0x54,0x18,0x00 } , /* e  */
    { 0x00,0x04,0x3E,0x05,0x01,0x00 } , /* f  */
    { 0x00,0x08,0x54,0x54,0x3C,0x00 } , /* g  */
    { 0x00,0x7F,0x04,0x04,0x78,0x00 } , /* h  */
    { 0x00,0x00,0x7D,0x00,0x00,0x00 } , /* i  */
    { 0x00,0x40,0x40,0x3D,0x00,0x00 } , /* j  */
    { 0x00,0x7F,0x10,0x28,0x44,0x00 } , /* k  */
    { 0x00,0x01,0x7F,0x00,0x00,0x00 } , /* l  */
    { 0x7C,0x04,0x7C,0x04,0x78,0x00 } , /* m  */
    { 0x00,0x7C,0x04,0x04,0x78,0x00 } , /* n  */
    { 0x00,0x38,0x44,0x44,0x38,0x00 } , /* o  */
    { 0x00,0x7C,0x14,0x14,0x08,0x00 } , /* p  */
    { 0x00,0x08,0x14,0x14,0x7C,0x00 } , /* q  */
    { 0x00,0x7C,0x08,0x04,0x04,0x00 } , /* r  */
    { 0x00,0x48,0x54,0x54,0x24,0x00 } , /* s  */
    { 0x00,0x04,0x3E,0x44,0x40,0x00 } , /* t  */
    { 0x00,0x3C,0x40,0x40,0x7C,0x00 } , /* u  */
    { 0x00,0x7C,0x20,0x10,0x0C,0x00 } , /* v  */
    { 0x1C,0x60,0x1C,0x60,0x1C,0x00 } , /* w  */
    { 0x00,0x6C,0x10,0x10,0x6C,0x00 } , /* x  */
    { 0x00,0x4C,0x50,0x30,0x1C,0x00 } , /* y  */
    { 0x00,0x44,0x64,0x54,0x4C,0x00 } , /* z  */
    { 0x00,0x08,0x36,0x41,0x41,0x00 } , /* {  */
    { 0x00,0x00,0x7F,0x00,0x00,0x00 } , /* |  */
    { 0x41,0x41,0x36,0x08,0x00,0x00 } , /* }  */
    { 0x08,0x04,0x08,0x10,0x08,0x00 } , /* ~  */
    { 0x00,0x00,0x00,0x00,0x00,0x00 }    /*null*/
};


extern uint8_t first_char_flag;

void reset_cursor()
{
    char_x = 0;
    char_y = 0;
    first_char_flag = 1;
}


void PutChar(uint8_t column,uint8_t row, int value)
{
        if(first_char_flag == 1)
        {
            SEGGER_RTT_printf(0, "char_x = %d, char_y = %d\n", char_x, char_y);
            first_char_flag = 0;
        }

        // internal font
        
        if(value == '\n') {
            char_x = 0;
            char_y = char_y + Y_height;
        }
        if ((value < 31) || (value > 127)) return;   // test char range
        if (char_x + X_width > width) {
            char_x = 0;
            char_y = char_y + Y_height;
            if (char_y >= height - Y_height) {
                char_y = 0;
            }
        }

        
        int i,j,w,lpx,lpy,k,l,xw;
        unsigned char Temp=0;
        j = 0; i = 0;
        w = X_width;
        FontSizeConvert(&lpx, &lpy);
        xw = X_width;
        
        for(i=0; i<xw; i++) {
            for ( l=0; l<lpx; l++) {
                Temp = font6x8[value-32][i];
                
                uint8_t line_length = 0;
                
                for(j=Y_height-1; j>=0; j--) {
                    
                    for (k=0; k<lpy; k++) {    
                        if( (Temp & 0x80)==0x80) {
                            line_length++;
                        }else if(line_length>0){
                            line(char_x+(i*lpx)+l, char_y+(((j+1)*lpy)-1)-k, char_x+(i*lpx)+l,  char_y+(((j+1)*lpy)-1)-k + line_length-1, Char_Color);
                            line_length = 0;
                        }
                        
                        
                    }
                    Temp = Temp << 1;
                }
            }
        }
        FontSizeConvert(&lpx, &lpy);
        char_x += (w*lpx);
}


uint16_t toRGB(uint16_t R,uint16_t G,uint16_t B)
{  
    uint16_t c;
    c = R >> 3;
    c <<= 6;
    c |= G >> 2;
    c <<= 5;
    c |= B >> 3;
    return c;
}

void line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint16_t color)
{
    if  ( x1 > width ) x1 = width;
    if  ( y1 > height ) y1 = height;
    if  ( x2 > width ) x2 = width;
    if  ( y2 > height ) y2 = height;

    unsigned char cmd[11]= { 0 };
    cmd[0] = GAC_FILL_ENABLE_DISABLE;
    cmd[1] = 0;      // fill 0, empty 0
    writeCommand_buf(&cmd, 2);
    cmd[0] = GAC_DRAW_LINE;
    cmd[1] = (unsigned char)x1;
    cmd[2] = (unsigned char)y1;
    cmd[3] = (unsigned char)x2;
    cmd[4] = (unsigned char)y2;
    cmd[5] = (unsigned char)(((color>>11)&0x1F)<<1);    // Blue
    cmd[6] = (unsigned char)((color>>5)&0x3F);          // Green
    cmd[7] = (unsigned char)((color&0x1F)<<1);          // Red

    writeCommand_buf(&cmd, 8);
    
}

void locate(uint8_t column, uint8_t row)
{
    char_x  = column;
    char_y = row;
}

void foreground(uint16_t color)
{
    Char_Color = color;
}
void background(uint16_t color)
{
    BGround_Color = color;
}

void SetFontSize(uint8_t Csize)
{
    chr_size = Csize;
}



void pixel(uint8_t x,uint8_t y,uint16_t Color)
{
    unsigned char cmd[7]= {Set_Column_Address,0x00,0x00,Set_Row_Address,0x00,0x00};
    if ((x>width)||(y>height)) return ;
    cmd[1] = (unsigned char)x;
    cmd[2] = (unsigned char)x;
    cmd[4] = (unsigned char)y;
    cmd[5] = (unsigned char)y;
    writeCommand_buf(&cmd, 6);
    writeCommand(Color);
}


int _getc()
{
    return -1;
}

uint8_t row()
{
    return char_y;
}
uint8_t column()
{
    return char_x;
}

void FontSizeConvert(int *lpx,int *lpy)
{
    switch( chr_size ) {
        case WIDE:
            *lpx=2;
            *lpy=1;
            break;
        case HIGH:
            *lpx=1;
            *lpy=2;
            break;
        case WH  :
            *lpx=2;
            *lpy=2;
            break;
        case WHx36  :
            *lpx=6;
            *lpy=6;
            break;
        case NORMAL:
        default:
            *lpx=1;
            *lpy=1;
            break;
    }
}
