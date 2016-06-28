#!/usr/bin/env python
# -*- coding: UTF-8 -*-
 

import serial
import threading
import time



class ComDev:
    def __init__(self):
       
        
    def open_com(self,port,baudrate):
        try:
            self.com = serial.Serial(port,baudrate)
        except serial.SerialException:#self.com = None
            print 'Open %s fail!' %com
    
	def com_isopen(self):
		if self.com.isOpen():
			return True
		
	def rec_data(self):	#接收的数据组
		#sign=True
		r_data=[]
		while True:   
			r_data = self.com.read(25)#send 25 char every time
			if r_data=""
					continue
			if r_data[0]=='beg' and r_data[-1]=='$':
				#rr_data=struct.unpack('B', r_data)
				data[0]=rr_data[1]
				data[1]=rr_data[2]
			
			if !self.com.isOpen():
				#sign=False
					break
			time.sleep(0.05)
			return data
			
	def send_data(self,Lspeed=00.0,Rspeed=00.0)
		#sign=True
		s_data=['beg','00.0','00.0','$']
		while True:
			s_data[1]=str(Lspeed)
			s_data[2]=str(Rspeed)
			if s_data=""
					continue
            self.com.write(s_data)
            if !self.com.isOpen():
					break
			time.sleep(0.05)
	
    def close_lisen_com(self):
        self.com.close()
        #self.com = None
        return False
	
	# Producer thread
	def ComThread()
		threads = []
		t1 = threading.Thread(target=rec_data)
		threads.append(t1)
		t2 = threading.Thread(target=send_data)
		threads.append(t2)
		
		
	if __name__ == '__main__':
		for t in threads:
			t.setDaemon(True)
			t.start()
		t.join()
	
    
   
            
        
 
        


