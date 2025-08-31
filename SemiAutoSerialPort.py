import serial
import serial.tools.list_ports

def semi_auto_serial_port(default_port = None):
    port_list = list(serial.tools.list_ports.comports())
    port_name_list = [port_info[0] for port_info in port_list]
    if len(port_list) == 0:
        if default_port:
            print(f"警告：串口列表为空，将返回默认串口[{default_port}]")
            return default_port
        else:
            print("警告：串口列表为空且未设置默认串口，将返回[COM1]")
            return "COM1"
    elif len(port_list) == 1:
        port_name = port_name_list[0]
        print(f"发现唯一串口[{port_name}]")
        if default_port and port_name != default_port:
            print(f"警告：默认串口[{default_port}]与发现串口[{port_name}]不一致，将返回发现串口")
        return port_name
    else:
        if default_port and default_port in port_name_list:
            print(f"发现默认串口[{default_port}]，将返回默认串口")
            return default_port
        if not default_port:
            print("警告：发现多个串口且未设置默认串口，请手动输入串口号")
        else:
            print("警告：发现多个串口且未发现默认串口，请手动输入串口号")
        print(f"发现串口列表：{port_list}")
        return input("输入选择的串口号（如COM1）：")
