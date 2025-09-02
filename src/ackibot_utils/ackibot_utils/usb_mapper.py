
from pyudev import Context, Device
import glob

vid_pid__2_class__map = {
	'1a86:7523' : ['Arduino'], # QinHeng Electronics CH340 serial converter
	'0403:6015' : ['Arduino'], # Future Technology Devices International, Ltd Bridge(I2C/SPI/UART/FIFO)
	'10c4:ea60' : ['LIDAR'], # Silicon Labs CP210x UART Bridge
	'067b:2303' : ['Debug UART', 'IMU'], # Prolific Technology, Inc. PL2303 Serial Port / Mobile Phone Data Cable
	'0403:6001' : ['Debug UART', 'IMU'], # Future Technology Devices International, Ltd FT232 Serial (UART) IC
}

class USB_Mapper:

	def __init__(self):
		self.table = {}

		patterns = [
			'/dev/ttyUSB*',
			'/dev/ttyACM*',
		]
		dev_files = sum(
			[glob.glob(p) for p in patterns],
			[]
		)

		context = Context()

		for dev_file in dev_files:
			dev = Device.from_device_file(context, dev_file)

			if False:
				print(dev_file)
				for k, v in dev.items():
					print(k, ' = ', v)
				print()

			#TODO Figure out port from this dev.get('ID_PATH')  =  pci-0000:00:14.0-usb-0:4.4:1.0
			vid = dev.get('ID_VENDOR_ID') #TODO ID_USB_VENDOR_ID
			pid = dev.get('ID_MODEL_ID')
			vid_pid = f'{vid}:{pid}'

			if vid_pid in vid_pid__2_class__map:
				list_of_classes = vid_pid__2_class__map[vid_pid]
				for c in list_of_classes:
					if not c in self.table:
						self.table[c] = []
					self.table[c].append(dev_file)
			else:
				raise RuntimeError(
					f'There is no {vid_pid} in vid_pid__2_class__map!'
				)
	def get_exactly_1_dev_of_class(self, class_name):
		c = self.table[class_name]
		if len(c) != 1:
			if len(c) == 0:
				raise RuntimeError(f'{class_name} not connected!')
			else:
				raise RuntimeError(f'More than 1 {class_name} connected!')
		else:
			return c[0]
			
		