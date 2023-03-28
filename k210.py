# 识别直线例程
#
# 这个例子展示了如何在图像中查找线条。对于在图像中找到的每个线对象，
# 都会返回一个包含线条旋转的线对象。

# 注意：线条检测是通过使用霍夫变换完成的：
# http://en.wikipedia.org/wiki/Hough_transform
# 请阅读以上关于“theta”和“rho”的更多信息。
00000000000
# find_lines（）找到无限长度的线。使用find_line_segments（）
# 来查找非无限线。

enable_lens_corr = False # turn on for straighter lines...打开以获得更直的线条…

from fpioa_manager import fm
import struct
import ustruct
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




#选择两个引脚，K210的引脚配置非常灵活，具体可以参考官方手册
fm.register(6, fm.fpioa.UART1_TX, force=True)
fm.register(7, fm.fpioa.UART1_RX, force=True)
fm.register(12, fm.fpioa.GPIO0)
LED_B = GPIO(GPIO.GPIO0, GPIO.OUT) #构建LED对象
LED_B.value(0) #点亮LED
#串口初始化
uart = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)
#sensor.reset()
#sensor.set_pixformat(sensor.GRAYSCALE) #灰度更快
#sensor.set_framesize(sensor.VGA)

#sensor.skip_frames(time = 2000)
#clock = time.clock()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.GRAYSCALE) #灰度更快
sensor.skip_frames(time = 2000)
#sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
#关闭白平衡
clock = time.clock()
lcd.init(freq=15000000)
lcd.direction(lcd.YX_RLDU)

#__________________________________________________________________
# 定时器的使用
# 定义定时器属性类
class timer_property():
    cnt     = 0                                             # 定时器计数值
    cnt_max = 0                                             # 定时器计数值上限
    period  = 0                                             # 定时器周期
    freq    = 0                                             # 定时器频率


# 定时器0 配置_______________________________________________________
# 定时器0 实例化类
timer0 = timer_property()                                   # 实例化定时器属性类 timer_property() 为 timer0
timer0.cnt_max = 9                                          # 设定 定时器0 的计数值上限为 9
timer0.period = 20                                         # 设定 定时器0 的周期为 100

# 定时器0 定义回调函数
def timer0_back(tim0):
    if timer0.cnt < timer0.cnt_max:                         # 若 定时器0 的计数值小于 定时器0 的计数值上限
        timer0.cnt = timer0.cnt + 1                         # 计数值自增
    else:
        timer0.cnt = 0                                      # 超出计数值上限 则计数值重置为0

# 定时器0 初始化
tim0 = Timer(Timer.TIMER0,                                  # 定时器编号 定时器0
            Timer.CHANNEL0,                                 # 定时器通道 通道0
            mode = Timer.MODE_PERIODIC,                     # 定时器模式 周期性
            unit = Timer.UNIT_MS,                           # 定时器周期单位 ms
            period = timer0.period,                         # 定时器周期 timer0.period 若 unit 为 Timer.UNIT_MS 则周期为 timer0.period ms
            callback = timer0_back)                         # 定时器触发中断后执行的回调函数 timer0_back




BINARY_VISIBLE = False # 首先执行二进制操作，以便您可以看到正在运行的线性回归...虽然可能会降低FPS。
THRESHOLD = (0, 70) # Grayscale threshold for dark things...
# 所有的线对象都有一个`theta（）`方法来获取它们的旋转角度。
# 您可以根据旋转角度来过滤线条。
#define num 5
min_degree = 0
max_degree = 179

min_degree0 = 170
max_degree0 = 10


min_degree90 = 70
max_degree90 = 110

min_rt_l_rho = 0
rho_rt_next = 0

parallel_rho=0
parallel_next=0


degree_0=[0,0,0,0,0]#窗口为5的滑动滤波
rho_0=[0,0,0,0,0]

degree0_num=0
rho0_num=0

degree_90=[0,0,0,0,0]#窗口为5的滑动滤波
rho_90=[0,0,0,0,0]

degree90_num=0
rho90_num=0

encoder_filter_index = 0
encoder_filter_index2 = 0

roi1=(36,27,108,94)
roi2=(71,24,53,89)
roi2=(64,31,220,210)
roi2=(29,57,250,172)
roi2=(287,0,342,90)
roi2=(20,0,200,250)
roi2=(43,0,193,241)
roi2=(32,4,191,237)
uart_idx=0
uart_buff=[1,2,3,0,0,0,0,0]
mode=0
def sending_data(cx,cy,cz):
    global uart;
    #frame=[0x2C,18,cx%0xff,int(cx/0xff),cy%0xff,int(cy/0xff),0x5B];
    #data = bytearray(frame)
    data = ustruct.pack("<bbhhhb",      #格式为俩个字符俩个短整型(2字节)
                   0x3f,                      #帧头1
                   0x22,                      #帧头2
                   int(cx*100), # up sample by 2   #数据1
                   int(cy*100), # up sample by 2    #数据2
                   int(cz*100),
                   0x1f
                   )
    uart.write(data)   #必须要传入一个字节数组
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



#print("mode:%d"%(mode))

def prosses_line(regression_rt):
    global encoder_filter_index
    global rho_90,degree_90
    rho0_num=abs(regression_rt.rho())
    theta=regression_rt.theta()
    if(theta<90):
        theta=theta+180
    magnitude=regression_rt.magnitude()


    print("liners:%f" %(rho0_num))  # 打印和中心位置的偏差
    img.draw_line(regression_rt.line(), color = (0,255 , 0))
    #img.draw_line(get_rt_l.line(), color = (0, 255, 0))
    #img.draw_line(get_parallel.line(), color = (0, 255, 0))


    #防止出现负值
    encoder_filter_index+=1



    degree_90[encoder_filter_index%5]=theta
    rho_90[encoder_filter_index%5]=abs(regression_rt.rho())


    #均值滤波 垂直
    sum = 0
    for k in degree_90:
        sum =sum+ k
    degree90_num= sum/5
    sum = 0

    for k in rho_90:
        sum =sum+ k
    rho90_num= sum/5
    sum = 0

    return [rho90_num,degree90_num]
    # print("FPS %f" % clock.fps())
    #print("degree0_num:%f" %(degree0_num))  # 打印和中心位置的偏差

    ##print("degree90_num:%f" %(degree90_num))  # 打印和中心位置的偏差


    #sending_data(256.798,123.123)

def prosses_line2(regression_parallel):
    global encoder_filter_index2,rho_0,degree_0
    rho0_num2=abs(regression_parallel.rho())
    theta2=regression_parallel.theta()
    magnitude2=regression_parallel.magnitude()
    print("liners2:%f" %(rho0_num2))  # 打印和中心位置的偏差
    img.draw_line(regression_parallel.line(), color = (0,255 , 0))

    encoder_filter_index2+=1
    if(regression_parallel.theta()<=min_degree0):
        degree_0[encoder_filter_index2%5]=180+regression_parallel.theta()
        rho_0[encoder_filter_index2%5]=abs(regression_parallel.rho())
    else:
        degree_0[encoder_filter_index2%5]=regression_parallel.theta()
        rho_0[encoder_filter_index2%5]=abs(regression_parallel.rho())


    #均值滤波 水平
    sum = 0
    for k in degree_0:
        sum =sum+ k
    degree0_num= sum/5
    sum = 0

    for k in rho_0:
        sum =sum+ k
    rho0_num= sum/5
    sum = 0


    return [rho0_num,degree0_num]
GRAYSCALE_THRESHOLD = (82,247)
# 所有线段都有 `x1()`, `y1()`, `x2()`, and `y2()` 方法来获得他们的终点
# 一个 `line()` 方法来获得所有上述的四个元组值，可用于 `draw_line()`.
#防止返回空值导致报错
while(True):
    img = sensor.snapshot()
    get_rt_l = img.get_regression([(255,255) if BINARY_VISIBLE else THRESHOLD])
    if (get_rt_l):break
get_parallel=get_rt_l
mode=86



while(True):
    clock.tick()
    get_uart_data()
    LED_B.value(0) #亮
    #print("mode:%d"%(mode))
    #if(mode>=83 and timer0.cnt == 0):
    if(mode>=83):
        print("mode:%d"%(mode))
        if(mode==83):#抓6色
            roi2=(32,4,191,237)
            roi2=(57,4,164,237)
            rt_roi_xywh=(0,67,50,174)#x不给
            rt_roi_xywh=(0,21,50,174)#x不给new
            parallel_roixywh=(0,0,240,150)#y不给
        if(mode==84):#bridge
            roi2=(30,59,215,182)
            roi2=(30,64,222,177)
            roi2=(30,90,255,150)
            rt_roi_xywh=(0,75,50,241)
            parallel_roixywh=(0,0,233,50)#y不给
        if(mode==85):#放置
            roi2=(32,4,191,237)
            rt_roi_xywh=(0,67,50,174)#x不给
            parallel_roixywh=(0,0,240,50)#y不给
        if(mode==86):#抓6xia
            roi2=(32,4,191,237)
            rt_roi_xywh=(0,8,50,224)#x不给
            parallel_roixywh=(0,0,251,150)#y不给
        img = sensor.snapshot()
        LED_B.value(1) #点亮LED
        #img.binary([GRAYSCALE_THRESHOLD])

        if enable_lens_corr: img.lens_corr(1.82) # for 2.8mm lens...

        # `threshold` controls how many lines in the image are found. Only lines with
        # edge difference magnitude sums greater than `threshold` are detected...

        # `threshold`控制从霍夫变换中监测到的直线。只返回大于或等于阈值的
        # 直线。应用程序的阈值正确值取决于图像。注意：一条直线的大小是组成
        # 直线所有索贝尔滤波像素大小的总和。

        # `theta_margin`和`rho_margin`控件合并相似的直线。如果两直线的
        # theta和ρ值差异小于边际，则它们合并。


        for l in img.find_lines(roi = roi2,threshold = 1000,theta_margin= 20, rho_margin = 40 ,x_stride=2, y_stride=1):
            if (min_degree90 <= l.theta()) and (l.theta() <= max_degree90):#判断垂直线 特征点
                rho_rt_next=l.rho()
                if(rho_rt_next<min_rt_l_rho):#试试最大值
                    min_rt_l_rho=rho_rt_next
                    get_rt_l=l
                #img.draw_line(l.line(), color = (255, 0, 0))
                # print(l)
            if (max_degree0 >= l.theta()) or (l.theta() >= min_degree0):#判断平行线 巡线  思路 先找最小像素值 之后合并 然后再找最短的线
                img.draw_line(l.line(), color = (255, 255, 0))
                parallel_next=l.rho()
                if(mode==86):#抓6色补丁 累了想不动了
                    #if(parallel_next>150):
                    if(parallel_next>180):
                        if(parallel_next<parallel_rho):
                            parallel_rho=parallel_next
                            get_parallel=l
                elif (parallel_next<parallel_rho):
                    parallel_rho=parallel_next
                    get_parallel=l


        #更新最小值
        parallel_rho=5000
        min_rt_l_rho=5000
        #img.draw_line(get_parallel.line(), color = (0, 255, 0))
        line_y= int((get_parallel.x1()+get_parallel.x2())/2)
        line_x=int((get_rt_l.y1()+get_rt_l.y2())/2)

        print("line_y:%f" %(line_y))  # 打印和中心位置的偏差
        rt_roi=(line_y-25,rt_roi_xywh[1],rt_roi_xywh[2],rt_roi_xywh[3])
        if(mode==83):
            parallel_roi=(parallel_roixywh[0],line_x-75,parallel_roixywh[2],parallel_roixywh[3])
        if(mode==84):
            parallel_roi=(parallel_roixywh[0],line_x-25,parallel_roixywh[2],parallel_roixywh[3])
        if(mode==85):
            parallel_roi=(parallel_roixywh[0],line_x-25,parallel_roixywh[2],parallel_roixywh[3])
        if(mode==86):
            parallel_roi=(parallel_roixywh[0],line_x-75,parallel_roixywh[2],parallel_roixywh[3])
        regression_rt = img.get_regression([(255,255) if BINARY_VISIBLE else THRESHOLD],roi = rt_roi)#有点问题
        regression_parallel= img.get_regression([(255,255) if BINARY_VISIBLE else THRESHOLD],roi = parallel_roi)
        #if (regression_rt): img.draw_line(regression_rt.line(), color = 127)
        if(regression_rt):
            [rho90_num,degree90_num]=prosses_line(regression_rt)
        if(regression_parallel):
            [rho0_num,degree0_num]=prosses_line2(regression_parallel)
        print("rho90_num:%f" %(rho90_num))  # 打印和中心位置的偏差
        print("rho0_num:%f" %(rho0_num))  # 打印和中心位置的偏差
        print("degree90_num:%f" %(degree90_num))  # 打印和中心位置的偏差
        #print("degree0_num:%f" %(degree0_num))  # 打印和中心位置的偏差
        sending_data(rho0_num,rho90_num,degree90_num)
