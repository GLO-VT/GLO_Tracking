# -*- coding: utf-8 -*-
"""
Created on Wed Jun 27 15:08:21 2018

@author: Bailey group PC
"""
from multiprocessing import Queue,Process
import time

def test(queue1,delay):
    while True:
        if queue1.empty() == True:
            #print('getting to hello')
            queue1.put(time.time())
        else:
            queue1.get()
            queue1.put(time.time())
            #print('inside queue empty')
        #time.sleep(0.5)

if __name__ == '__main__':        
    queue1=Queue(1)   
    delay=1  
    test_p = Process(name='test',target=test,args=(queue1,
                                                   delay))
    
    test_p.start()
    
    t_time=time.time()
    while time.time()-t_time < 15:
        #print('queue empty',SS1_queue.empty())
        if queue1.empty() == False:
            print(queue1.get()-time.time())
        else:
            print('queue empty')
        time.sleep(0.01)
            