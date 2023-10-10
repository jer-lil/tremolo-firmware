import serial
import matplotlib.pyplot as plt
import numpy as np

serialPort = serial.Serial(
    port="COM4", 
    baudrate=1_000_000, 
    bytesize=8, 
    timeout=2, 
    stopbits=serial.STOPBITS_ONE,
    parity=serial.PARITY_ODD
)

def get_next_table():
    # Read until start of frame
    bytes_in = serialPort.read_until(b'\x01\x0D\x0A', 2100)
    # Next byte is table index
    table_index = int.from_bytes(serialPort.read(1), "little", signed="False") 
    return table_index


def get_data():
    # Throw out start of text byte
    serialPort.read(1)
    # Read 2048 bytes
    bytes_in = serialPort.read(2048)
    ints_in = [int.from_bytes(bytes_in[i:i+2], "little", signed="False") for i in range(0, len(bytes_in), 2)]
    return ints_in
    


if __name__ == "__main__":
    x = np.linspace(0, 1023, 1024)
    y = get_data()
    plt.ion()
    fig = plt.figure()
    fig.set_size_inches(8, 6)
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    ax.set_ylim([0, 1023+16])
    ax.set_xlim([-16, 1023+16])

    line_a_lo, = ax.plot(x, y, 'r-', label='CH A - High Frequencies')
    line_a_hi, = ax.plot(x, y, 'g-', label='CH A - Low Frequencies')
    line_b_lo, = ax.plot(x, y, 'b-', label='CH B - High Frequencies')
    line_b_hi, = ax.plot(x, y, 'y-', label='CH B - Low Frequencies')
    
    plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    plt.tight_layout()
    plt.show()

    table_index = get_next_table()
    new_data = get_data()
    while table_index != 3:
        table_index = get_next_table()
        new_data = get_data()
    
    while 1:

        for i in range(4):
            table_index = get_next_table()
            
            if table_index == 0:
                new_data_a_lo = get_data()
            elif table_index == 1:
                new_data_a_hi = get_data()
            elif table_index == 2:
                new_data_b_lo = get_data()
            elif table_index == 3:
                new_data_b_hi = get_data()
            else:
                print(f'index = <table_index>, expected value in range [0, 3]')
        
        line_a_lo.set_ydata(new_data_a_lo)
        line_a_hi.set_ydata(new_data_a_hi)
        line_b_lo.set_ydata(new_data_b_lo)
        line_b_hi.set_ydata(new_data_b_hi)   
        fig.canvas.draw()
        fig.canvas.flush_events()



