/*
 * Copyright (c) 2016, dmytro.fedorchenko
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package logic140;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import gnu.io.*;
/**
 *
 * @author dmytro.fedorchenko
 */
public class Generator {
    private String                      m_PortName = "COM1";
    private CommPortIdentifier          m_portId = null;
    private InputStream                 m_InputStream = null;
    private OutputStream                m_OutputStream = null;
    private SerialPort                  m_SerialPort = null;
    private int                         m_Speed = 115200;
    private final String                c_RespondStr = "EMAT";
    private int                         m_WaitBytesNum = 10;
    private int                         m_Offset = 0;
    private int                         m_Width = 10;
    private boolean                     m_IsOpen = false;
    private final byte                  c_EndStr[] = {(byte)0x0D};
   
    Generator()
    {
    }
    
    public void setPortName(String portName)
    {
        m_PortName = portName;
    }
    
    public String getPortName()
    {
        return m_PortName;
    }
    
    public void setSpeed(int speed){
        if (speed > 0){
            m_Speed = speed;
        }
    }
    
    public int getSpeed(){
        return m_Speed;
    }
    
    private CommPortIdentifier findPortByName(String portName)
    {
        CommPortIdentifier  rportId = null;
        Enumeration portList = CommPortIdentifier.getPortIdentifiers();

	while (portList.hasMoreElements()) {
	    CommPortIdentifier portId = (CommPortIdentifier) portList.nextElement();
	    if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
		if (portId.getName().equals(portName)) {
                    rportId = portId;
                    break;
		} 
	    } 
	} 
        
        return rportId;
    }
    
    private boolean sendCommand(String cmd){
        return sendCommand(cmd.getBytes());
    }
    private boolean sendCommand(byte[] cmd){
       try {
           m_InputStream.skip(m_InputStream.available());
           m_OutputStream.write(cmd);
           m_OutputStream.write(c_EndStr);
           m_WaitBytesNum =  c_RespondStr.length() + 18*cmd.length + c_EndStr.length + 13;
       } catch (IOException e) {
           return false;
       }
       
       return true;
    }
    
    private boolean readCheckRespond(){
        byte[] readBuffer = new byte[m_WaitBytesNum];
        
        try {
            if (m_InputStream.available() > m_WaitBytesNum) {
                m_InputStream.read(readBuffer);
                String str = new String (readBuffer, StandardCharsets.UTF_8);
                return str.contains(c_RespondStr);
            }
        } catch (IOException e) {
            return false;
        }
        
        return false;
    }
    
    private boolean sendCmdB(String cmd){
        if (sendCommand(cmd) != true) {
            return false;
        }
        
        return readCheckRespond();
    }
    
    public int getOffset() {
        return m_Offset;
    }
    
    public int getWidth() {
        return m_Width;
    }
    
    public boolean setOffset(int offset){
        if (offset < 0) {
            return false;
        }
        if (m_IsOpen) {
            if (sendCmdB("offset " + offset) != true) {
                return false;
            }
        }
        
        m_Offset  = offset;
      
        return true;
    }
    
    public boolean setWidth(int width){
        if (width < 0) { 
            return false;
        }
        
        if (m_IsOpen) {
            if (sendCmdB("width " + width) != true) {
                return false;
            }
        }
        
        m_Width = width;
        
        return true;
    }
    
    private boolean openDev(){
        close();
          
        m_portId = findPortByName(m_PortName);
        
        if (m_portId == null){
            return false;
        }
        
        try {
            m_SerialPort = (SerialPort) m_portId.open("GenProg", m_Speed);
        } catch (PortInUseException e){
            close();
            return false;
        }
        
        try {
            m_InputStream = m_SerialPort.getInputStream();
            m_OutputStream = m_SerialPort.getOutputStream();
        } catch (IOException e) {
            close();
            return false;
        }
        
        if (sendCmdB(new String(c_EndStr)) != true) {
            close();
            return false;
        }

        m_IsOpen = true;
        
        return true;
    }
    
    public boolean open() {
        if (openDev() != true) {
            return false;
        } 
        
        if (setOffset(m_Offset) != true) {
            close();
            return false;
        }
        
        if (setWidth(m_Width) != true) {
            close();
            return false;
        }
        
        return true;
    }
    
    public boolean start() {
        
        if (m_IsOpen != true) {
            if (open() != true) {
                return false;
            }
        }
        
        if (sendCmdB("start") != true) {
            close();
            return false;
        }
        
        return true;
    }
    
    public boolean stop() {
        if (sendCmdB("stop") != true) {
            close();
            return false;
        }
        
        return true;
    }
    
    public void close()
    {
        if (m_SerialPort != null){
            m_SerialPort.close();
            m_SerialPort = null;
        }
        
        m_IsOpen = false;
        m_portId = null;
        m_InputStream = null;
        m_OutputStream = null;
    }
    
}
