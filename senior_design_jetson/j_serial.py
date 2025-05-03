import serial
import time

#init port
def init():
    #init serial
    serial_port = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    #wait for port to init
    time.sleep(1)
    #return the port
    return serial_port

#close port and print confirmation
def close_port(serial_port):
    serial_port.close()
    print("port closed")

#transmit and receive data
def Tx_Rx(serial_port, data, r, w):
    try:
        #get data from buffer
        if (serial_port.in_waiting > 0) and r == True:
            rx_data = Rx(serial_port)
            return rx_data
        if w == True:
            Tx(serial_port, data)
    except Exception as exception_error:
        print("error occurred. Exiting")
        print("Error: " +str(exception_error))

#recieve data in waiting
def Rx(serial_port):
    #limiting the address to be 8 bytes
    data = serial_port.read(8)
    return data

#transmit data. data should be a string without carriage return
def Tx(serial_port, data):
    serial_port.write(data.encode())


#main function
def main():
    port = init()
    while(1):
        Tx_Rx(port, "100jrdyr", False, True)
        print("100jrdyr")
        if(port.in_waiting > 0):
            rx = Tx_Rx(port, "", True, False)
            print(rx)
        time.sleep(1)

if __name__ == "__main__":
    main()

