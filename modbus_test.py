from pymodbus.client.sync import ModbusTcpClient
addr = 5000

client = ModbusTcpClient("192.168.178.112")
client.connect()
reg = client.read_holding_registers(addr)
print('%d: 0x%x' % (addr, reg.getRegister(0)))

# client.write_register(100, 0x1234)
