#include "lcd.h"
//#include "delay.h"
#include "font.h"

//LCD的画笔颜色和背景色
uint16_t POINT_COLOR=0x0000; //画笔颜色
uint16_t BACK_COLOR=0xFFFF;  //背景色

//管理LCD重要参数
_lcd_dev lcddev;
extern SRAM_HandleTypeDef hsram1;

static uint32_t LCD_Pow(uint8_t m,uint8_t n);
//static uint16_t LCD_BGR2RGB(uint16_t c);
static void getLcdDevID(void);
static void LCD_Init_NT35310(void);
//static void opt_delay(uint8_t i);

/**
 * @brief LCD_WR_REG 写寄存器函数
 * @param regval 寄存器值
 */
void LCD_WR_REG(__IO uint16_t regval)
{
  HAL_SRAM_Write_16b(&hsram1, (uint32_t*)&LCD_WR_Reg, (uint16_t *)&regval, 1);
}

/**
 * @brief LCD_WR_DATA 写LCD数据
 * @param data 要写入的值
 */
void LCD_WR_DATA(__IO uint16_t data)
{
  HAL_SRAM_Write_16b(&hsram1, (uint32_t*)&LCD_WR_Data, (uint16_t *)&data, 1);
}

/**
 * @brief LCD_RD_DATA 读LCD数据
 * @return 读到的值
 */
uint16_t LCD_RD_DATA(void)
{
  __IO uint16_t ram; //防止被优化
  HAL_SRAM_Read_16b(&hsram1, (uint32_t*)&LCD_WR_Data, (uint16_t *)&ram, 1);
  return ram;
}

/**
 * @brief LCD_WriteReg 写寄存器
 * @param LCD_Reg 寄存器地址
 * @param LCD_RegValue 要写入的数据
 */
void LCD_WriteReg(__IO uint16_t LCD_Reg, __IO uint16_t LCD_RegValue)
{
  LCD_WR_REG(LCD_Reg);
  LCD_WR_DATA(LCD_RegValue);
}

/**
 * @brief LCD_ReadReg 读寄存器
 * @param LCD_Reg 寄存器地址
 * @return 读到的数据
 */
uint16_t LCD_ReadReg(__IO uint16_t LCD_Reg)
{
  LCD_WR_REG(LCD_Reg);  //写入要读的寄存器序号
  HAL_Delay(1);//delay_us(5);
  return LCD_RD_DATA(); //返回读到的值
}

/**
 * @brief LCD_WriteRAM_Prepare 开始写GRAM
 */
void LCD_WriteRAM_Prepare(void)
{
  LCD_WR_REG(lcddev.wramcmd);
}

/**
 * @brief LCD_WriteRAM LCD写GRAM
 * @param RGB_Code 颜色值
 */
void LCD_WriteRAM(uint16_t RGB_Code)
{
  LCD_WR_DATA(RGB_Code);
}

//初始化lcd
//lcd型号：NT35310
void LCD_Init(void)
{
  getLcdDevID(); //读id
  if(lcddev.id!=0X5310)
    lcddev.id=0X5310;

  if(lcddev.id==0x5310)
  {
    //重新配置写时序控制寄存器的时序
    hsram1.Extended->BWTR[6]&=~(0XF<<0); //地址建立时间（ADDSET）清零
    hsram1.Extended->BWTR[6]&=~(0XF<<8); //数据保存时间清零
    hsram1.Extended->BWTR[6]|=3<<0;      //地址建立时间（ADDSET）为3个HCLK =18ns
    hsram1.Extended->BWTR[6]|=2<<8;      //数据保存时间为6ns*3个HCLK=18ns
    LCD_Init_NT35310();
    LCD_Display_Dir(1);//横屏
  }
}

/**
 * @brief LCD_DisplayOn LCD开启显示
 */
void LCD_DisplayOn(void)
{
  if(lcddev.id==0X5310)
    LCD_WR_REG(0X29);	//开启显示
}

/**
 * @brief LCD_DisplayOff LCD关闭显示
 */
void LCD_DisplayOff(void)
{
  if(0X5310)
    LCD_WR_REG(0X28);	//关闭显示
}

/**
 * @brief LCD_Clear 清屏函数
 * @param color 要清屏的填充色
 */
void LCD_Clear(uint16_t color)
{
  uint32_t index=0;
  uint32_t totalpoint=lcddev.width;
  totalpoint*=lcddev.height; 			//得到总点数
  if((lcddev.id==0X6804)&&(lcddev.dir==1))//6804横屏的时候特殊处理
  {
    lcddev.dir=0;
    lcddev.setxcmd=0X2A;
    lcddev.setycmd=0X2B;
    LCD_SetCursor(0x00,0x0000);		//设置光标位置
    lcddev.dir=1;
    lcddev.setxcmd=0X2B;
    lcddev.setycmd=0X2A;
  }
  else LCD_SetCursor(0x00,0x0000);	//设置光标位置
  LCD_WriteRAM_Prepare();     		//开始写入GRAM
  for(index=0; index<totalpoint; index++)
  {
    LCD_WR_DATA(color);
  }
}

/**
 * @brief LCD_SetCursor 设置光标位置
 * @param Xpos 横坐标
 * @param Ypos 纵坐标
 */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  if(lcddev.id==0X5310)
  {
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(Xpos>>8);
    LCD_WR_DATA(Xpos&0XFF);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(Ypos>>8);
    LCD_WR_DATA(Ypos&0XFF);
  }
}

/**
 * @brief LCD_DrawPoint 画点
 * @param x 坐标
 * @param y
 */
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
  LCD_SetCursor(x,y);		//设置光标位置
  LCD_WriteRAM_Prepare();	//开始写入GRAM
  LCD_WR_DATA(POINT_COLOR);
}

/**
 * @brief LCD_Fast_DrawPoint 快速画点
 * @param x 坐标
 * @param y
 * @param color 颜色
 */
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
  //if(lcddev.dir==1)x=lcddev.width-1-x;//横屏其实就是调转x,y坐标
  LCD_WR_REG(lcddev.setxcmd);
  LCD_WR_DATA(x>>8);
  LCD_WR_DATA(x&0XFF);
  LCD_WR_REG(lcddev.setycmd);
  LCD_WR_DATA(y>>8);
  LCD_WR_DATA(y&0XFF);

  LCD_WriteRAM_Prepare();
  LCD_WR_DATA(color);
}

/**
 * @brief LCD_ReadPoint 读取个某点的颜色值
 * @param x 坐标
 * @param y
 * @return 此点的颜色
 */
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
  __IO uint16_t r,g=0,b=0;
  if(x>=lcddev.width||y>=lcddev.height)return 0;	//超过了范围,直接返回
  LCD_SetCursor(x,y);
  if(lcddev.id==0X5310)LCD_WR_REG(0X2E);//发送读GRAM指令

  (void)LCD_RD_DATA();									//dummy Read
  //opt_delay(2);
  r=LCD_RD_DATA();  		  						//实际坐标颜色
  if(lcddev.id==0X5310)	//NT35310要分2次读出
  {
    //opt_delay(2);
    b=LCD_RD_DATA();
    g=r&0XFF;		//对于5310,第一次读取的是RG的值,R在前,G在后,各占8位
    g<<=8;
  }
  return (((r>>11)<<11)|((g>>10)<<5)|(b>>11));//ILI9341/NT35310/NT35510需要公式转换一下
}

/**
 * @brief LCD_Draw_Circle 在指定位置画一个指定大小的圆
 * @param x0 (x,y):中心点
 * @param y0
 * @param r 半径
 */
void LCD_Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r)
{
  //int a,b;
  uint16_t a,b;
  int di;
  a=0;
  b=r;
  di=3-(r<<1);             //判断下个点位置的标志
  while(a<=b)
  {
    LCD_DrawPoint(x0+a,y0-b);             //5
    LCD_DrawPoint(x0+b,y0-a);             //0
    LCD_DrawPoint(x0+b,y0+a);             //4
    LCD_DrawPoint(x0+a,y0+b);             //6
    LCD_DrawPoint(x0-a,y0+b);             //1
    LCD_DrawPoint(x0-b,y0+a);
    LCD_DrawPoint(x0-a,y0-b);             //2
    LCD_DrawPoint(x0-b,y0-a);             //7
    a++;
    //使用Bresenham算法画圆
    if(di<0)di +=4*a+6;
    else
    {
      di+=10+4*(a-b);
      b--;
    }
  }
}

/**
 * @brief LCD_DrawLine 画线
 * @param x1 起点坐标
 * @param y1
 * @param x2 起点坐标
 * @param y2
 */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  uint16_t t;
  int xerr=0,yerr=0,delta_x,delta_y,distance;
  int incx,incy;
  uint16_t uRow,uCol;
  delta_x=x2-x1; //计算坐标增量
  delta_y=y2-y1;
  uRow=x1;
  uCol=y1;
  if(delta_x>0)incx=1; //设置单步方向
  else if(delta_x==0)incx=0;//垂直线
  else
  {
    incx=-1;
    delta_x=-delta_x;
  }
  if(delta_y>0)incy=1;
  else if(delta_y==0)incy=0;//水平线
  else
  {
    incy=-1;
    delta_y=-delta_y;
  }
  if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴
  else distance=delta_y;
  for(t=0; t<=distance+1; t++ ) //画线输出
  {
    LCD_DrawPoint(uRow,uCol);//画点
    xerr+=delta_x ;
    yerr+=delta_y ;
    if(xerr>distance)
    {
      xerr-=distance;
      uRow+=(uint16_t)incx;
    }
    if(yerr>distance)
    {
      yerr-=distance;
      uCol+=(uint16_t)incy;
    }
  }
}

/**
 * @brief LCD_DrawRectangle 画矩形
 * @param x1 矩形的对角坐标
 * @param y1
 * @param x2
 * @param y2
 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  LCD_DrawLine(x1,y1,x2,y1);
  LCD_DrawLine(x1,y1,x1,y2);
  LCD_DrawLine(x1,y2,x2,y2);
  LCD_DrawLine(x2,y1,x2,y2);
}

/**
 * @brief LCD_Fill 在指定区域内填充单个颜色
 * @param sx (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)
 * @param sy
 * @param ex
 * @param ey
 * @param color 要填充的颜色
 */
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color)
{
  uint16_t i,j;
  uint16_t xlen;
  xlen=(ex-sx)+1;
  for(i=sy; i<=ey; i++)
  {
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(sx>>8);
    LCD_WR_DATA(sx&0xff);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(i>>8);
    LCD_WR_DATA(i&0xff);
    LCD_WriteRAM_Prepare();
    for(j=0; j<xlen; j++)LCD_WR_DATA(color);	//显示颜色
  }
}

/**
 * @brief LCD_Color_Fill 在指定区域内填充指定颜色块
 * @param sx (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)
 * @param sy
 * @param ex
 * @param ey
 * @param color 要填充的颜色
 */
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,const uint16_t *color)
{
  uint16_t height,width;
  uint16_t i,j;
  width=(ex-sx)+1; 			//得到填充的宽度
  height=(ey-sy)+1;			//高度
  for(i=0; i<height; i++)
  {
    LCD_SetCursor(sx,sy+i);   	//设置光标位置
    LCD_WriteRAM_Prepare();     //开始写入GRAM
    for(j=0; j<width; j++)LCD_WR_DATA(color[i*width+j]); //写入数据
  }
}

/**
 * @brief LCD_ShowChar 在指定位置显示一个字符
 * @param x 起始坐标
 * @param y
 * @param num 要显示的字符:" "--->"~"
 * @param size 字体大小 12/16/24
 * @param mode 叠加方式(1)还是非叠加方式(0)
 */
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode)
{
  uint8_t temp,t1,t;
  uint16_t y0=y;
  //得到字体一个字符对应点阵集所占的字节数
  uint8_t csize=(size/8+((size%8)?1:0))*(size/2);
  //设置窗口
  //得到偏移后的值
  num=num-' ';
  for(t=0; t<csize; t++)
  {
    //调用1206字体
    if(size==12)temp=asc2_1206[num][t];
    //调用1608字体
    else if(size==16)temp=asc2_1608[num][t];
    //调用2412字体
    else if(size==24)temp=asc2_2412[num][t];
    //没有的字库
    else return;
    for(t1=0; t1<8; t1++)
    {
      if(temp&0x80)LCD_Fast_DrawPoint(x,y,POINT_COLOR);
      else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
      temp<<=1;
      y++;
      //超区域了
      if(y>=lcddev.height)return;
      if((y-y0)==size)
      {
        y=y0;
        x++;
        //超区域了
        if(x>=lcddev.width)return;
        break;
      }
    }
  }
}

/**
 * @brief LCD_ShowNum 显示数字,高位为0,则不显示
 * @param x 起点坐标
 * @param y
 * @param num 数值(0~4294967295)
 * @param len 数字的位数
 * @param size 字体大小
 */
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size)
{
  uint8_t t,temp;
  uint8_t enshow=0;
  for(t=0; t<len; t++)
  {
    temp=(num/LCD_Pow(10,len-t-1))%10;
    if(enshow==0&&t<(len-1))
    {
      if(temp==0)
      {
        LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
        continue;
      }
      else enshow=1;

    }
    LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0);
  }
}

/**
 * @brief LCD_ShowxNum 显示数字,高位为0,还是显示
 * @param x 起点坐标
 * @param y
 * @param num 数值(0~999999999)
 * @param len 长度(即要显示的位数)
 * @param size 字体大小
 * @param mode
 * [7]:0,不填充;1,填充0
 * [6:1]:保留
 * [0]:0,非叠加显示;1,叠加显示.
 */
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode)
{
  uint8_t t,temp;
  uint8_t enshow=0;
  for(t=0; t<len; t++)
  {
    temp=(num/LCD_Pow(10,len-t-1))%10;
    if(enshow==0&&t<(len-1))
    {
      if(temp==0)
      {
        if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,mode&0X01);
        else LCD_ShowChar(x+(size/2)*t,y,' ',size,mode&0X01);
        continue;
      }
      else enshow=1;

    }
    LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode&0X01);
  }
}

/**
 * @brief LCD_ShowString 显示字符串
 * @param x 起点坐标
 * @param y
 * @param width 区域大小
 * @param height
 * @param size 字体大小
 * @param p 字符串起始地址
 */
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p)
{
  uint8_t x0=x;
  width+=x;
  height+=y;
  while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
  {
    if(x>=width)
    {
      x=x0;
      y+=size;
    }
    if(y>=height)break;//退出
    LCD_ShowChar(x,y,*p,size,0);
    x+=size/2;
    p++;
  }
}

/**
 * @brief LCD_Scan_Dir 设置LCD的自动扫描方向
 * 注意:其他函数可能会受到此函数设置的影响(尤其是9341/6804这两个奇葩),
 * 所以,一般设置为L2R_U2D即可,如果设置为其他扫描方式,可能导致显示不正常.
 * 9320/9325/9328/4531/4535/1505/b505/8989/5408/9341/5310/5510等IC已经实际测试
 * dir:0~7,代表8个方向(具体定义见lcd.h)
 * @param dir
 */
void LCD_Scan_Dir(uint8_t dir)
{
  uint16_t regval=0;
  uint16_t dirreg=0;
  uint16_t temp;
  if(lcddev.dir==1)
  {
    switch(dir)//方向转换
    {
    case 0:
      dir=6;
      break;
    case 1:
      dir=7;
      break;
    case 2:
      dir=4;
      break;
    case 3:
      dir=5;
      break;
    case 4:
      dir=1;
      break;
    case 5:
      dir=0;
      break;
    case 6:
      dir=3;
      break;
    case 7:
      dir=2;
      break;
    default:
      break;
    }
  }
  if(lcddev.id==0X5310)
  {
    switch(dir)
    {
    case L2R_U2D://从左到右,从上到下
      //regval|=(0<<7)|(0<<6)|(0<<5);
      break;
    case L2R_D2U://从左到右,从下到上
      //regval|=(1<<7)|(0<<6)|(0<<5);
      regval|=(1<<7);
      break;
    case R2L_U2D://从右到左,从上到下
      //regval|=(0<<7)|(1<<6)|(0<<5);
      regval|=(1<<6);
      break;
    case R2L_D2U://从右到左,从下到上
      //regval|=(1<<7)|(1<<6)|(0<<5);
      regval|=(1<<7)|(1<<6);
      break;
    case U2D_L2R://从上到下,从左到右
      //regval|=(0<<7)|(0<<6)|(1<<5);
      regval|=(1<<5);
      break;
    case U2D_R2L://从上到下,从右到左
      //regval|=(0<<7)|(1<<6)|(1<<5);
      regval|=(1<<6)|(1<<5);
      break;
    case D2U_L2R://从下到上,从左到右
      //regval|=(1<<7)|(0<<6)|(1<<5);
      regval|=(1<<7)|(1<<5);
      break;
    case D2U_R2L://从下到上,从右到左
      regval|=(1<<7)|(1<<6)|(1<<5);
      break;
    default:
      break;
    }
    dirreg=0X36;
    LCD_WriteReg(dirreg,regval);
    if(regval&0X20)
    {
      if(lcddev.width<lcddev.height)//交换X,Y
      {
        temp=lcddev.width;
        lcddev.width=lcddev.height;
        lcddev.height=temp;
      }
    }
    else
    {
      if(lcddev.width>lcddev.height)//交换X,Y
      {
        temp=lcddev.width;
        lcddev.width=lcddev.height;
        lcddev.height=temp;
      }
    }
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(0);
    LCD_WR_DATA(0);
    //LCD_WR_DATA((lcddev.width-1)>>8);
    //LCD_WR_DATA((lcddev.width-1)&0XFF);
    LCD_WR_DATA(lcddev.width>>8);
    LCD_WR_DATA(lcddev.width&0XFF);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(0);
    LCD_WR_DATA(0);
//    LCD_WR_DATA((lcddev.height-1)>>8);
//    LCD_WR_DATA((lcddev.height-1)&0XFF);
    LCD_WR_DATA(lcddev.height>>8);
    LCD_WR_DATA(lcddev.height&0XFF);
  }
}

/**
 * @brief LCD_Display_Dir 设置LCD显示方向
 * @param dir 0,竖屏；1,横屏
 */
void LCD_Display_Dir(uint8_t dir)
{
  if(dir==0)			//竖屏
  {
    lcddev.dir=0;	//竖屏
    if(lcddev.id==0X5310)
    {
      lcddev.wramcmd=0X2C;
      lcddev.setxcmd=0X2A;
      lcddev.setycmd=0X2B;
      lcddev.width=320;
      lcddev.height=480;
    }
  }
  else 				//横屏
  {
    lcddev.dir=1;	//横屏
    if(lcddev.id==0X5310)
    {
      lcddev.wramcmd=0X2C;
      lcddev.setxcmd=0X2A;
      lcddev.setycmd=0X2B;
      lcddev.width=480;
      lcddev.height=320;
    }
  }
  LCD_Scan_Dir(DFT_SCAN_DIR);	//默认扫描方向
}

/**
 * @brief LCD_Set_Window 设置窗口,并自动设置画点坐标到窗口左上角(sx,sy).
 * @param sx 窗口起始坐标(左上角)
 * @param sy
 * @param width 窗口宽度和高度,必须大于0!!
 * @param height
 * 68042,横屏时不支持窗口设置!!
 */
void LCD_Set_Window(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
  uint8_t hsareg,heareg,vsareg,veareg;
  uint16_t hsaval,heaval,vsaval,veaval;
  width=sx+width-1;
  height=sy+height-1;
  if(lcddev.id==0X9341||lcddev.id==0X5310||lcddev.id==0X6804)//6804横屏不支持
  {
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(sx>>8);
    LCD_WR_DATA(sx&0XFF);
    LCD_WR_DATA(width>>8);
    LCD_WR_DATA(width&0XFF);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(sy>>8);
    LCD_WR_DATA(sy&0XFF);
    LCD_WR_DATA(height>>8);
    LCD_WR_DATA(height&0XFF);
  }
  else if(lcddev.id==0X5510)
  {
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(sx>>8);
    LCD_WR_REG(lcddev.setxcmd+1);
    LCD_WR_DATA(sx&0XFF);
    LCD_WR_REG(lcddev.setxcmd+2);
    LCD_WR_DATA(width>>8);
    LCD_WR_REG(lcddev.setxcmd+3);
    LCD_WR_DATA(width&0XFF);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(sy>>8);
    LCD_WR_REG(lcddev.setycmd+1);
    LCD_WR_DATA(sy&0XFF);
    LCD_WR_REG(lcddev.setycmd+2);
    LCD_WR_DATA(height>>8);
    LCD_WR_REG(lcddev.setycmd+3);
    LCD_WR_DATA(height&0XFF);
  }
  else	//其他驱动IC
  {
    if(lcddev.dir==1)//横屏
    {
      //窗口值
      hsaval=sy;
      heaval=height;
      vsaval=lcddev.width-width-1;
      veaval=lcddev.width-sx-1;
    }
    else
    {
      hsaval=sx;
      heaval=width;
      vsaval=sy;
      veaval=height;
    }
    hsareg=0X50;
    heareg=0X51;//水平方向窗口寄存器
    vsareg=0X52;
    veareg=0X53;//垂直方向窗口寄存器
    //设置寄存器值
    LCD_WriteReg(hsareg,hsaval);
    LCD_WriteReg(heareg,heaval);
    LCD_WriteReg(vsareg,vsaval);
    LCD_WriteReg(veareg,veaval);
    LCD_SetCursor(sx,sy);	//设置光标位置
  }
}

///**
// * @brief LCD_BGR2RGB 从ILI93xx读出的数据为GBR格式，而我们写入的时候为RGB格式。
// * 通过该函数转换
// * @param c GBR格式的颜色值
// * @return RGB格式的颜色值
// */
//static uint16_t LCD_BGR2RGB(uint16_t c)
//{
//  uint16_t  r,g,b,rgb;
//  b=c&0x1f;
//  g=(c>>5)&0x3f;
//  r=(c>>11)&0x1f;
//  rgb=(uint16_t)((b<<11)+(g<<5)+r);
//  return(rgb);
//}

///**
// * @brief opt_delay 当mdk -O1时间优化时需要设置
// * @param i 延时i
// */
//static void opt_delay(uint8_t i)
//{
//  while(i--);
//}

/**
 * @brief LCD_Pow m^n函数
 * @param m
 * @param n
 * @return 返回值:m^n次方.
 */
static uint32_t LCD_Pow(uint8_t m,uint8_t n)
{
  uint32_t result=1;
  while(n--)result*=m;
  return result;
}

/**
 * @brief getLcdDevID
 * @return
 */
static void getLcdDevID(void)
{
  HAL_Delay(50); // delay 50 ms
  LCD_WriteReg(0x0000,0x0001);
  HAL_Delay(50); // delay 50 ms
  lcddev.id = LCD_ReadReg(0x0000);
  if(lcddev.id<0XFF||lcddev.id==0XFFFF||lcddev.id==0X9300)//读到ID不正确,新增lcddev.id==0X9300判断，因为9341在未被复位的情况下会被读成9300
  {
    //尝试9341 ID的读取
    LCD_WR_REG(0XD3);
    lcddev.id=LCD_RD_DATA();	//dummy read
    lcddev.id=LCD_RD_DATA(); 	//读到0X00
    lcddev.id=LCD_RD_DATA();   	//读取93
    lcddev.id<<=8;
    lcddev.id|=LCD_RD_DATA();  	//读取41
    if(lcddev.id!=0X9341)		//非9341,尝试是不是6804
    {
      LCD_WR_REG(0XBF);
      lcddev.id=LCD_RD_DATA();//dummy read
      lcddev.id=LCD_RD_DATA();//读回0X01
      lcddev.id=LCD_RD_DATA();//读回0XD0
      lcddev.id=LCD_RD_DATA();//这里读回0X68
      lcddev.id<<=8;
      lcddev.id|=LCD_RD_DATA();//这里读回0X04
      if(lcddev.id!=0X6804)	//也不是6804,尝试看看是不是NT35310
      {
        LCD_WR_REG(0XD4);
        lcddev.id=LCD_RD_DATA();	//dummy read
        lcddev.id=LCD_RD_DATA();	//读回0X01
        lcddev.id=LCD_RD_DATA();	//读回0X53
        lcddev.id<<=8;
        lcddev.id|=LCD_RD_DATA();	//这里读回0X10
        if(lcddev.id!=0X5310)		//也不是NT35310,尝试看看是不是NT35510
        {
          LCD_WR_REG(0XDA00);
          lcddev.id=LCD_RD_DATA();//读回0X00
          LCD_WR_REG(0XDB00);
          lcddev.id=LCD_RD_DATA();//读回0X80
          lcddev.id<<=8;
          LCD_WR_REG(0XDC00);
          lcddev.id|=LCD_RD_DATA();//读回0X00
          if(lcddev.id==0x8000)lcddev.id=0x5510;//NT35510读回的ID是8000H,为方便区分,我们强制设置为5510
        }
      }
    }
  }
}

static void LCD_Init_NT35310(void)
{
  LCD_WR_REG(0xED);
  LCD_WR_DATA(0x01);
  LCD_WR_DATA(0xFE);

  LCD_WR_REG(0xEE);
  LCD_WR_DATA(0xDE);
  LCD_WR_DATA(0x21);

  LCD_WR_REG(0xF1);
  LCD_WR_DATA(0x01);
  LCD_WR_REG(0xDF);
  LCD_WR_DATA(0x10);

  //VCOMvoltage//
  LCD_WR_REG(0xC4);
  LCD_WR_DATA(0x8F);	  //5f

  LCD_WR_REG(0xC6);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xE2);
  LCD_WR_DATA(0xE2);
  LCD_WR_DATA(0xE2);
  LCD_WR_REG(0xBF);
  LCD_WR_DATA(0xAA);

  LCD_WR_REG(0xB0);
  LCD_WR_DATA(0x0D);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x0D);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x11);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x19);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x21);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x2D);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x3D);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x5D);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x5D);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xB1);
  LCD_WR_DATA(0x80);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x8B);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x96);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xB2);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x02);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x03);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xB3);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xB4);
  LCD_WR_DATA(0x8B);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x96);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA1);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xB5);
  LCD_WR_DATA(0x02);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x03);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x04);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xB6);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xB7);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x3F);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x5E);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x64);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x8C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xAC);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xDC);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x70);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x90);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xEB);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xDC);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xB8);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xBA);
  LCD_WR_DATA(0x24);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC1);
  LCD_WR_DATA(0x20);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x54);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xFF);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC2);
  LCD_WR_DATA(0x0A);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x04);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC3);
  LCD_WR_DATA(0x3C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x3A);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x39);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x37);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x3C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x36);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x32);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x2F);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x2C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x29);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x26);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x24);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x24);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x23);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x3C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x36);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x32);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x2F);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x2C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x29);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x26);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x24);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x24);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x23);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC4);
  LCD_WR_DATA(0x62);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x05);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x84);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xF0);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x18);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA4);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x18);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x50);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x0C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x17);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x95);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xF3);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xE6);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC5);
  LCD_WR_DATA(0x32);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x44);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x65);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x76);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x88);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC6);
  LCD_WR_DATA(0x20);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x17);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x01);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC7);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC8);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xC9);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xE0);
  LCD_WR_DATA(0x16);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x1C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x21);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x36);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x46);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x52);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x64);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x7A);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x8B);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x99);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA8);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xB9);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xC4);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xCA);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD2);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD9);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xE0);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xF3);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xE1);
  LCD_WR_DATA(0x16);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x1C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x22);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x36);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x45);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x52);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x64);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x7A);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x8B);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x99);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA8);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xB9);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xC4);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xCA);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD2);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD8);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xE0);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xF3);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xE2);
  LCD_WR_DATA(0x05);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x0B);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x1B);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x34);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x44);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x4F);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x61);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x79);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x88);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x97);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA6);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xB7);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xC2);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xC7);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD1);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD6);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xDD);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xF3);
  LCD_WR_DATA(0x00);
  LCD_WR_REG(0xE3);
  LCD_WR_DATA(0x05);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x1C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x33);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x44);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x50);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x62);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x78);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x88);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x97);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA6);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xB7);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xC2);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xC7);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD1);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD5);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xDD);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xF3);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xE4);
  LCD_WR_DATA(0x01);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x01);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x02);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x2A);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x3C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x4B);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x5D);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x74);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x84);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x93);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA2);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xB3);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xBE);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xC4);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xCD);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD3);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xDD);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xF3);
  LCD_WR_DATA(0x00);
  LCD_WR_REG(0xE5);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x02);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x29);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x3C);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x4B);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x5D);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x74);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x84);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x93);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xA2);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xB3);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xBE);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xC4);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xCD);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xD3);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xDC);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xF3);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xE6);
  LCD_WR_DATA(0x11);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x34);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x56);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x76);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x77);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x66);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x88);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x99);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xBB);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x99);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x66);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x55);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x55);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x45);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x43);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x44);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xE7);
  LCD_WR_DATA(0x32);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x55);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x76);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x66);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x67);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x67);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x87);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x99);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xBB);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x99);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x77);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x44);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x56);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x23);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x33);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x45);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xE8);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x99);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x87);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x88);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x77);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x66);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x88);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xAA);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0xBB);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x99);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x66);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x55);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x55);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x44);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x44);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x55);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xE9);
  LCD_WR_DATA(0xAA);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0x00);
  LCD_WR_DATA(0xAA);

  LCD_WR_REG(0xCF);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xF0);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x50);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xF3);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0xF9);
  LCD_WR_DATA(0x06);
  LCD_WR_DATA(0x10);
  LCD_WR_DATA(0x29);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0x3A);
  LCD_WR_DATA(0x55);	//66

  LCD_WR_REG(0x11);
  HAL_Delay(1);//delay_ms(100);
  LCD_WR_REG(0x29);
  LCD_WR_REG(0x35);
  LCD_WR_DATA(0x00);

  LCD_WR_REG(0x51);
  LCD_WR_DATA(0xFF);
  LCD_WR_REG(0x53);
  LCD_WR_DATA(0x2C);
  LCD_WR_REG(0x55);
  LCD_WR_DATA(0x82);
  LCD_WR_REG(0x2c);
}

