import time
import threading
import random

def Thermal():
    last_diff = 0
    diff_sum = 0
    k_p = 0.05
    k_i = 5
    k_d = 0.025
    
    while True:
        r_t = (desired_temp - 32) * 5/9
        y_t = get_sensor_data()
        time.sleep(0.5)
        diff = r_t - y_t
        diff_sum += diff
        P = k_p * diff
        I = k_i * diff_sum
        D = k_d * last_diff
        u_t = P + I + D
        SetVout(u_t)
        last_diff = diff
    
def SetVout(u_t):
    global Vout 
    Vout = u_t
    return
    
def get_sensor_data() -> float:
    global y_t
    y_t = 0.99*y_t + 0.01*Vout
    return y_t

if __name__ == '__main__':
    FIlEOUT = open('log.txt', 'w')
    global Vout
    global y_t
    global desired_temp
    y_t = (80 - 32) * 5/9
    Vout = y_t
    desired_temp = 70
    x = threading.Thread(target=Thermal, daemon=True)
    x.start()
    for i in range(100000):
        vout_log = str((9/5 * y_t) + 32) + '\n'
        print(vout_log)
        FIlEOUT.write(vout_log)
        time.sleep(0.5)
        if(i % 1000 == 0):
            desired_temp = random.uniform(60.00, 80.00)
            msg = 'with new tempurature: ' + str(desired_temp) + '\n'
            print(msg)
            FIlEOUT.write(msg)
            
    FIlEOUT.close()      