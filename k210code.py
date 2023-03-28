from fpioa_manager import fm

import ustruct
import struct
import machine
import sensor
import lcd
import time
import image
import sys
import utime
from Maix import GPIO
from Maix import freq
from fpioa_manager import fm
from board import board_info
from machine import UART
from machine import Timer
import math
import lcd,image

from machine import Timer,PWM
import utime
#PWM 通过定时器配置，接到 IO17 引脚
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
LED_PWM = PWM(tim, freq=3000, duty=0, pin=17)




ring_thresholds =[(5,50,20,60,10,50),(70,100,-50,-20,5,40),(-10,30,0,40,-70,-10)]#颜色 1red 2绿 3蓝 下
graythreshold=[(50,100)]

thresholds=[(28,75,25,70,0,50),(50,90,-50,-20,5,50),(10,60,-10,40,-70,-20)]#颜色 1red 2绿 3蓝 shang
thresholdsL=[(21, 83, 34, 84, 22, 70),(70,100,-60,-15,18,57),(10, 61, -10, 73, -70, -37)]
thresholdsH=[(21, 83, 34, 84, 22, 70),(70,100,-50,-20,10,40),(10, 61, -10, 73, -70, -37)]


thresholdsH=[(21, 83, 34, 84, 22, 70),(2, 40, -96, 20, 2, 59),(0, 90, -48, 95, -128, -29)]
thresholdsL=[(21, 83, 34, 84, 22, 70),(63, 100, -128, -7, -30, 120),(0, 90, -48, 95, -128, -29)]
thresholdsH=[(21, 83, 34, 84, 22, 70),(0, 62, -55, -8, -24, 106),(0, 90, -48, 95, -128, -29)]
#thresholdsH=[(21, 83, 34, 84, 22, 70),(38, 98, -58, -23, 7, 90),(10, 61, -10, 73, -70, -37)]#新的颜色
thresholdsH = [(30, 83, 47, 127, -128, 127), # generic_red_thresholds
              (0, 100, -125, -18, -70, 73), # generic_green_thresholds
              (0, 100, -20, 126, -128, -40)] # generic_blue_thresholds
thresholdsL = [(30, 83, 47, 127, -128, 127), # generic_red_thresholds
                (19, 93, -128, -26, -128, 127), # generic_green_thresholds
                (0, 100, -20, 126, -128, -40)] # generic_blue_thresholds
thresholdsL = [(30, 83, 47, 127, -128, 127), # generic_red_thresholds
              (0, 100, -125, -18, -70, 73), # generic_green_thresholds
              (0, 100, -20, 126, -128, -40)] # generic_blue_thresholds
#选择两个引脚，K210的引脚配置非常灵活，具体可以参考官方手册
fm.register(6, fm.fpioa.UART1_TX, force=True)
fm.register(7, fm.fpioa.UART1_RX, force=True)
fm.register(12, fm.fpioa.GPIO0)
LED_B = GPIO(GPIO.GPIO0, GPIO.OUT) #构建LED对象
LED_B.value(0) #点亮LED
lcd.init(freq=15000000)
lcd.direction(lcd.YX_RLDU)
#串口初始化
uart = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)
#sensor.reset()
#sensor.set_pixformat(sensor.GRAYSCALE) #灰度更快
#sensor.set_framesize(sensor.VGA)

#sensor.skip_frames(time = 2000)
##
#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.set_vflip(1)   #后置模式
#sensor.set_auto_gain(False)  # 必须关闭此功能，以防止图像冲洗…
#sensor.skip_frames(30)


#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.set_vflip(True) #垂直翻转

#sensor.reset()#初始化摄像头
#sensor.set_pixformat(sensor.RGB565)#设置为彩色RGB模式
#sensor.set_framesize(sensor.QVGA)#设置图像的大小  sensor.QVGA为常量320*240
##sensor.set_windowing(320,240))#在当前画面中取出一块你想要处理的窗口，如果不写x和y坐标，roi会默认居中。
#sensor.skip_frames(time = 2000)#跳过n帧  ，指定time则跳过2000ms，等摄像头稳定再继续

#sensor.set_auto_gain(False) #自动增益开启（True）或者关闭（False）。在使用颜色追踪时，需要关闭自动增益。
#sensor.set_hmirror(0) #水平翻转

#自动增益：
#摄像机输出的视频信号必须达到电视传输规定的标准电平，即,为了能在不同的景物照度条件下都能输出的标准视频信号，
#必须使放大器的增益能够在较大的范围内进行调节。这种增益调节通常都是通过检测视频信号的平均电平而自动完成的，
#实现此功能的电路称为自动增益控制电路，简称AGC电路。具有AGC功能的摄像机，在低照度时的灵敏度会有所提高，
#但此时的噪点也会比较明显。这是由于信号和噪声被同时放大的缘故。

#sensor.set_auto_whitebal(False) # 自动白平衡开启（True）或者关闭（False）。在使用颜色追踪时，需要关闭自动白平衡。
#白平衡：
#它指的是在图像处理的过程中, 对原本材质为白色的物体的图像进行色彩还原, 去除外部光源色温的影响, 使其在照片上也显示白色。


#sensor.set_auto_exposure(1)                                 # 设置自动曝光
##sensor.set_auto_exposure(0, exposure=120000)               # 设置手动曝光 曝光时间 120000 us

#sensor.set_auto_gain(0, gain_db = 17)                       # 设置画面增益 17 dB 影响实时画面亮度
#sensor.set_auto_whitebal(0, rgb_gain_db = (0,0,0))          # 设置RGB增益 0 0 0 dB 影响画面色彩呈现效果 在 K210 上无法调节增益 初步判定是感光元件 ov2640 无法支持
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(0) #水平翻转
sensor.skip_frames(time = 2000)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 2000)
clock = time.clock()

text= '1 2 3 + 3 2 1'
#typ 0x01 :二维码 0x02颜色识别
uart_idx=0
uart_buff=[0,0,0,0,0,0,0,0]
mode=0

def sending_data(typ,cx,cy):
    global uart;
    #frame=[0x2C,18,cx%0xff,int(cx/0xff),cy%0xff,int(cy/0xff),0x5B];
    #data = bytearray(frame)
    data = ustruct.pack("<bbbbbbbbbb",      #格式为俩个字符俩个短整型(2字节)
                   0x3f,                      #帧头1
                   0x22,                      #帧头2
                   typ,                      #帧头2
                   cx[0], # up sample by 2   #数据1
                   cx[1], # up sample by 2   #数据1
                   cx[2], # up sample by 2   #数据1
                   cy[0], # up sample by 2   #数据1
                   cy[1], # up sample by 2   #数据1
                   cy[2], # up sample by 2   #数据1
                   0x1f
                   )
    uart.write(data);   #必须要传入一个字节数组

def get_uart_data():
    global uart;
    global uart_idx
    global uart_buff
    global mode
    if uart.any():  #进行串口数据的接收
        res=uart.read(1)  #表示为读取一个十六进制数,这里的uart必须是例化的
        res = struct.unpack('B', res)
        res = hex(res)
        res = int(res)
        uart_buff[uart_idx]=res
        uart_idx=uart_idx+1
        if(uart_idx>5):
            uart_idx=0
        for ix in range(5):
            if(uart_buff[ix]==81 and uart_buff[ix+2]==61):#没法判断hex类型 只能换成int 牛逼
                mode=uart_buff[ix+1]
                #print("mode:%d"%(mode))
                uart_buff=[0,0,0,0,0,0,0,0]

def wedges_H(img):#第二曾
    roi2=(0,3,319,73)
    location=[[0,0],[0,0],[0,0]]
    #img = sensor.snapshot()

    i=0

    for blob in img.find_blobs([thresholdsH[0]],roi=roi2,pixels_threshold=200, merge=True):
    #for blob in img.find_blobs([thresholdsH[0]],roi=roi2,pixels_threshold=200, area_threshold=200, merge=True):
        if blob.area()<4000 :
            location[0]=[blob.cx(),blob.cy()]
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.
            cx(), blob.cy())
            i+=1
    for blob in img.find_blobs([thresholdsH[1]],roi=roi2,pixels_threshold=200, merge=True):
        if blob.area()<4000 :
            location[1]=[blob.cx(),blob.cy()]
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            i+=1
    for blob in img.find_blobs([thresholdsH[2]],roi=roi2,pixels_threshold=200, merge=True):
        if blob.area()<4000 :
            location[2]=[blob.cx(),blob.cy()]
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            i+=1
    if i==3:
        add=(location[0][1]+location[1][1]+location[2][1])/3
        if((add-location[0][1]<15 or add-location[0][1]>-15)and\
            (add-location[1][1]<15 or add-location[1][1]>-15)and\
            (add-location[2][1]<15 or add-location[2][1]>-15)):
            if location[0][0]<location[1][0]<location[2][0]:
                return [1,2,3]
            if location[1][0]<location[0][0]<location[2][0]:
                return [2,1,3]
            if location[2][0]<location[1][0]<location[0][0]:
                return [3,2,1]
            if location[0][0]<location[2][0]<location[1][0]:
                return [1,3,2]
            if location[1][0]<location[2][0]<location[0][0]:
                return [2,3,1]
            if location[2][0]<location[0][0]<location[1][0]:
                return [3,1,2]
    return [0]
def wedges_L(img):
    roi2=(0,77,318,86)
    location=[[0,0],[0,0],[0,0]]
    #img = sensor.snapshot()
    i=0
    for blob in img.find_blobs([thresholdsL[0]],roi=roi2,pixels_threshold=200, merge=True):
        if blob.area()<4000 :
            location[0]=[blob.cx(),blob.cy()]
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            i+=1
    for blob in img.find_blobs([thresholdsL[1]],roi=roi2,pixels_threshold=200, merge=True):
        if blob.area()<4000 :
            location[1]=[blob.cx(),blob.cy()]
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            i+=1
    for blob in img.find_blobs([thresholdsL[2]],roi=roi2,pixels_threshold=200, merge=True):
        if blob.area()<4000 :
            location[2]=[blob.cx(),blob.cy()]
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            i+=1
    if i==3:
        add=(location[0][1]+location[1][1]+location[2][1])/3
        if((add-location[0][1]<15 or add-location[0][1]>-15)and\
            (add-location[1][1]<15 or add-location[1][1]>-15)and\
            (add-location[2][1]<15 or add-location[2][1]>-15)):
            if location[0][0]<location[1][0]<location[2][0]:
                return [1,2,3]
            if location[1][0]<location[0][0]<location[2][0]:
                return [2,1,3]
            if location[2][0]<location[1][0]<location[0][0]:
                return [3,2,1]
            if location[0][0]<location[2][0]<location[1][0]:
                return [1,3,2]
            if location[1][0]<location[2][0]<location[0][0]:
                return [2,3,1]
            if location[2][0]<location[0][0]<location[1][0]:
                return [3,1,2]
    return [0]


flag=1
text=''
textup= '1 2 3'
textdown='3 2 1'
mode=82
sendbuff_H=[0]
sendbuff_L=[0]

while(True):
    clock.tick()
    get_uart_data()
    print("mode:%d"%(mode))
    #img = sensor.snapshot()
    if(mode==86):
        #sensor.reset()
        #sensor.set_pixformat(sensor.RGB565)
        #sensor.set_framesize(sensor.QVGA)
        #sensor.set_hmirror(0) #水平翻转
        #sensor.skip_frames(time = 2000)
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_hmirror(0) #水平翻转
        sensor.skip_frames(time = 2000)
        sensor.set_auto_whitebal(False)
        sensor.skip_frames(time = 2000)
        mode=80
    if(mode==81):
        LED_PWM.duty(40)
        while(1):
           img = sensor.snapshot()
           buff_H=wedges_H(img)
           buff_L=wedges_L(img)
           if buff_H!=[0]and buff_L!=[0]:
               print(buff_H)
               print(buff_L)
               sending_data(0x02,buff_H,buff_L)
               break
           get_uart_data()
           if(mode!=81):
              break
        while(1):
            LED_PWM.duty(0)
            get_uart_data()
            utime.sleep(0.1)
            sending_data(0x02,buff_H,buff_L)
            if(mode!=81):
               break

    if(mode==82):
        LED_PWM.duty(0)
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_auto_exposure(1)
        sensor.set_hmirror(0) #水平翻转

        # 设置自动曝光
        #sensor.set_auto_exposure(0, exposure=120000)               # 设置手动曝光 曝光时间 120000 us

        #sensor.set_auto_gain(0, gain_db = 17)                       # 设置画面增益 17 dB 影响实时画面亮度
        #sensor.set_auto_whitebal(0, rgb_gain_db = (0,0,0))          # 设置RGB增益 0 0 0 dB 影响画面色彩呈现效果 在 K210 上无法调节增益 初步判定是感光元件 ov2640 无法支持

        #sensor.skip_frames(30)
        while(1):
            img = sensor.snapshot()
            #lcd.display(img)
            for code in img.find_qrcodes():
                text=code[4]
                print(code)
                textup=text[0:3]
                textdown=text[4:7]
                code1=[int(textup[0]),int(textup[1]),int(textup[2])]
                code2=[int(textdown[0]),int(textdown[1]),int(textdown[2])]
                print(code1)
                print(code2)
                sending_data(0x01,code1,code2)
                utime.sleep(0.1)
                sending_data(0x01,code1,code2)
                utime.sleep(0.1)
                sending_data(0x01,code1,code2)
                utime.sleep(0.1)
                sending_data(0x01,code1,code2)
                utime.sleep(0.1)
                sending_data(0x01,code1,code2)
                img.draw_string(50, 10,textup +' H', scale=10)
                img.draw_string(50, 120,textdown+'  ML', scale=10)
                lcd.display(img)
            get_uart_data()
            if(mode!=82):
                sensor.reset()
                sensor.set_pixformat(sensor.RGB565)
                sensor.set_framesize(sensor.QVGA)
                sensor.set_hmirror(0) #水平翻转
                sensor.skip_frames(time = 2000)
                break
