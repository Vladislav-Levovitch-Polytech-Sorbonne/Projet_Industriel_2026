from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter
from openpyxl.worksheet.datavalidation import DataValidation
from datetime import date

# 创建工作簿
wb = Workbook()
wb.remove(wb.active)

# 创建3个工作表
ws_overview = wb.create_sheet('Overview', 0)
ws_io = wb.create_sheet('IO_Signals', 1)
ws_hw = wb.create_sheet('Hardware_Interfaces', 2)

# 定义样式
header_fill = PatternFill(start_color='1F4E78', end_color='1F4E78', fill_type='solid')
header_font = Font(bold=True, color='FFFFFF', size=11)
normal_font = Font(size=10)
app_fill = PatternFill(start_color='D9E1F2', end_color='D9E1F2', fill_type='solid')
rte_fill = PatternFill(start_color='C6E0B4', end_color='C6E0B4', fill_type='solid')
bsw_fill = PatternFill(start_color='FFE699', end_color='FFE699', fill_type='solid')
hw_fill = PatternFill(start_color='E2EFDA', end_color='E2EFDA', fill_type='solid')
in_fill = PatternFill(start_color='E2EFDA', end_color='E2EFDA', fill_type='solid')
out_fill = PatternFill(start_color='FCE4D6', end_color='FCE4D6', fill_type='solid')

thin_border = Border(
    left=Side(style='thin'),
    right=Side(style='thin'),
    top=Side(style='thin'),
    bottom=Side(style='thin')
)

center_align = Alignment(horizontal='center', vertical='center')
left_align = Alignment(horizontal='left', vertical='center')

# ==================== Overview工作表 ====================
ws = ws_overview

# 项目标题
ws['A1'] = 'CoVAPSy - Course de Voitures Autonomes Paris-Saclay'
ws['A1'].font = Font(bold=True, size=14)
ws['A1'].alignment = left_align
ws.merge_cells('A1:D1')

# 副标题
ws['A2'] = 'Bilan I/O - System Integration Document'
ws['A2'].font = Font(bold=True, size=12, color='1F4E78')
ws.merge_cells('A2:D2')

# 基本信息
ws['A4'] = 'Version:'
ws['B4'] = 'v1.0'
ws['A5'] = 'Date:'
ws['B5'] = '2025-11-25'
ws['A6'] = 'Project Stage:'
ws['B6'] = 'Early Planning'
ws['A7'] = 'Maintainer:'
ws['B7'] = 'Integration Team'

for row in [4, 5, 6, 7]:
    ws[f'A{row}'].font = Font(bold=True)

# 系统架构概述
ws['A9'] = 'System Architecture (4 Layers):'
ws['A9'].font = Font(bold=True, size=11)

ws['A10'] = 'Layer'
ws['B10'] = 'Description'
ws['C10'] = 'Components'
for cell in ['A10', 'B10', 'C10']:
    ws[cell].font = header_font
    ws[cell].fill = header_fill
    ws[cell].alignment = center_align
    ws[cell].border = thin_border

data = [
    ['APP', 'Application Layer - ROS Nodes', 'Perception, Planning, Control'],
    ['RTE', 'Runtime Environment - ROS Middleware', 'ROS 2 DDS Topics & Services'],
    ['BSW', 'Basic Software - Drivers & Firmware', 'LiDAR/Camera Drivers, STM32 Firmware'],
    ['HW', 'Hardware Layer', 'RPi 4, STM32, Sensors, Actuators']
]

fills = [app_fill, rte_fill, bsw_fill, hw_fill]
for i, row_data in enumerate(data, start=11):
    ws[f'A{i}'] = row_data[0]
    ws[f'B{i}'] = row_data[1]
    ws[f'C{i}'] = row_data[2]
    for col in ['A', 'B', 'C']:
        ws[f'{col}{i}'].border = thin_border
        ws[f'{col}{i}'].alignment = left_align
    ws[f'A{i}'].fill = fills[i-11]
    ws[f'A{i}'].alignment = center_align
    ws[f'A{i}'].font = Font(bold=True)

# 主要组件汇总
ws['A16'] = 'Main Components Summary:'
ws['A16'].font = Font(bold=True, size=11)

ws['A17'] = 'Category'
ws['B17'] = 'Component'
ws['C17'] = 'Specification'
ws['D17'] = 'Qty'
for cell in ['A17', 'B17', 'C17', 'D17']:
    ws[cell].font = header_font
    ws[cell].fill = header_fill
    ws[cell].alignment = center_align
    ws[cell].border = thin_border

components = [
    ['Compute', 'Raspberry Pi', '4 Model B', '1'],
    ['Compute', 'STM32', 'L432KC Nucleo', '1'],
    ['Compute', 'ESC', 'Motor Controller', '1'],
    ['Sensor', 'LiDAR', 'RPLIDAR A2M8', '1'],
    ['Sensor', 'Camera', 'Depth Camera', '1'],
    ['Sensor', 'IMU', 'Inertial Measurement Unit', '1'],
    ['Sensor', 'Accelerometer', 'Adafruit Triple-Axis', '1'],
    ['Sensor', 'Distance Sensor', 'SHARP 2Y0A21', '4'],
    ['Actuator', 'Motor', 'Brushless (TT02)', '1'],
    ['Actuator', 'Servo', 'Reely RS-610WP MG', '1'],
    ['Power', 'Battery', 'NiMH 7.2V 3000mAh', '1'],
    ['Power', 'Power Distribution', 'PDB 5V/12V', '1'],
    ['Safety', 'Emergency Stop', 'Physical Button', '1']
]

for i, comp in enumerate(components, start=18):
    ws[f'A{i}'] = comp[0]
    ws[f'B{i}'] = comp[1]
    ws[f'C{i}'] = comp[2]
    ws[f'D{i}'] = comp[3]
    for col in ['A', 'B', 'C', 'D']:
        ws[f'{col}{i}'].border = thin_border
        ws[f'{col}{i}'].alignment = left_align
    ws[f'D{i}'].alignment = center_align

# 工作表导航
ws['A32'] = 'Worksheet Navigation:'
ws['A32'].font = Font(bold=True, size=11)

ws['A33'] = 'Sheet Name'
ws['B33'] = 'Content'
for cell in ['A33', 'B33']:
    ws[cell].font = header_font
    ws[cell].fill = header_fill
    ws[cell].border = thin_border

nav_data = [
    ['IO_Signals', 'Complete I/O signal definitions with layer, direction, protocol'],
    ['Hardware_Interfaces', 'Physical hardware connections, pinouts, communication parameters']
]

for i, nav in enumerate(nav_data, start=34):
    ws[f'A{i}'] = nav[0]
    ws[f'B{i}'] = nav[1]
    for col in ['A', 'B']:
        ws[f'{col}{i}'].border = thin_border
        ws[f'{col}{i}'].alignment = left_align

# 设置列宽
ws.column_dimensions['A'].width = 18
ws.column_dimensions['B'].width = 35
ws.column_dimensions['C'].width = 35
ws.column_dimensions['D'].width = 8

print('Overview sheet completed')

# ==================== I/O Signals工作表 ====================
ws = ws_io

# 列标题
headers = [
    'Signal ID', 'Signal Name', 'Layer', 'Direction', 'Source Component',
    'Target Component', 'Signal Type', 'Unit', 'Min Value', 'Max Value',
    'Frequency', 'Protocol', 'Port/Pin', 'Trigger Condition', 'Notes'
]

for col, header in enumerate(headers, start=1):
    cell = ws.cell(row=1, column=col)
    cell.value = header
    cell.font = header_font
    cell.fill = header_fill
    cell.alignment = center_align
    cell.border = thin_border

# I/O信号数据
io_data = [
    # 传感器信号
    ['IO-S001', 'LiDAR_Distance_Array', 'APP', 'IN', 'LiDAR_Driver', 'Perception_Node', 'array[float]', 'mm', '0', '25000', '20Hz', 'USB', '/dev/ttyUSB0', 'Continuous', '360-degree scan'],
    ['IO-S002', 'Camera_Frame', 'APP', 'IN', 'Camera_Driver', 'Perception_Node', 'image', 'N/A', 'N/A', 'N/A', '30Hz', 'USB', '/dev/video0', 'Continuous', 'RGB/H.264'],
    ['IO-S003', 'IMU_Accel_Data', 'BSW', 'IN', 'STM32_Firmware', 'RTE', 'struct', 'm/s^2', '-40', '40', '100Hz', 'I2C', 'STM32-PB8/PB9', 'Continuous', 'Triple-axis accelerometer'],
    ['IO-S004', 'IMU_Gyro_Data', 'BSW', 'IN', 'STM32_Firmware', 'RTE', 'struct', 'deg/s', '-2000', '2000', '100Hz', 'I2C', 'STM32-PB8/PB9', 'Continuous', 'Triple-axis gyroscope'],
    ['IO-S005', 'Distance_Sensor_FL', 'BSW', 'IN', 'STM32_Firmware', 'Safety_Monitor', 'uint16', 'cm', '10', '80', '50Hz', 'GPIO', 'STM32-PA0', 'Continuous', 'Front-left sharp sensor'],
    ['IO-S006', 'Distance_Sensor_FR', 'BSW', 'IN', 'STM32_Firmware', 'Safety_Monitor', 'uint16', 'cm', '10', '80', '50Hz', 'GPIO', 'STM32-PA1', 'Continuous', 'Front-right sharp sensor'],
    ['IO-S007', 'Distance_Sensor_RL', 'BSW', 'IN', 'STM32_Firmware', 'Safety_Monitor', 'uint16', 'cm', '10', '80', '50Hz', 'GPIO', 'STM32-PA2', 'Continuous', 'Rear-left sharp sensor'],
    ['IO-S008', 'Distance_Sensor_RR', 'BSW', 'IN', 'STM32_Firmware', 'Safety_Monitor', 'uint16', 'cm', '10', '80', '50Hz', 'GPIO', 'STM32-PA3', 'Continuous', 'Rear-right sharp sensor'],

    # 执行器信号
    ['IO-A001', 'Motor_PWM_Command', 'BSW', 'OUT', 'Control_Node', 'STM32_Firmware', 'uint16', '%', '0', '100', '200Hz', 'UART', 'STM32-PA5', 'After planning', 'Throttle control'],
    ['IO-A002', 'Servo_Angle_Command', 'BSW', 'OUT', 'Control_Node', 'STM32_Firmware', 'float', 'deg', '-45', '45', '50Hz', 'UART', 'STM32-PA6', 'After planning', 'Steering control'],

    # 安全和监控信号
    ['IO-E001', 'Emergency_Stop_Signal', 'HW', 'IN', 'E-Stop_Button', 'Safety_Monitor', 'bool', 'N/A', '0', '1', '100Hz', 'GPIO', 'RPi-GPIO27', 'Hardware trigger', 'Active high stops all'],
    ['IO-E002', 'Battery_Voltage', 'BSW', 'IN', 'Power_Module', 'Safety_Monitor', 'float', 'V', '0', '12', '10Hz', 'ADC', 'STM32-PA4', 'Continuous', 'Under-voltage warning at 6.0V'],

    # ROS2通信信号
    ['IO-C001', 'Perception_Output_Topic', 'RTE', 'OUT', 'Perception_Node', 'Planning_Node', 'custom_msg', 'N/A', 'N/A', 'N/A', '20Hz', 'DDS', 'ROS2 Topic', 'After perception', 'Environment representation'],
    ['IO-C002', 'Planning_Output_Topic', 'RTE', 'OUT', 'Planning_Node', 'Control_Node', 'custom_msg', 'N/A', 'N/A', 'N/A', '20Hz', 'DDS', 'ROS2 Topic', 'After planning', 'Trajectory waypoints'],
    ['IO-C003', 'Control_Feedback_Topic', 'RTE', 'IN', 'STM32_Firmware', 'Control_Node', 'custom_msg', 'N/A', 'N/A', 'N/A', '100Hz', 'DDS', 'ROS2 Topic', 'Continuous', 'Actuator state feedback'],

    # UART通信信号
    ['IO-U001', 'UART_Command_Frame', 'BSW', 'OUT', 'RPI4', 'STM32', 'binary', 'N/A', 'N/A', 'N/A', '200Hz', 'UART', '/dev/ttyAMA0', 'Real-time', 'Motor+Servo commands'],
    ['IO-U002', 'UART_Telemetry_Frame', 'BSW', 'IN', 'STM32', 'RPI4', 'binary', 'N/A', 'N/A', 'N/A', '100Hz', 'UART', '/dev/ttyAMA0', 'Real-time', 'IMU+sensors data']
]

# 填充数据
layer_fills = {'APP': app_fill, 'RTE': rte_fill, 'BSW': bsw_fill, 'HW': hw_fill}
dir_fills = {'IN': in_fill, 'OUT': out_fill, 'INOUT': PatternFill(start_color='FFFFCC', end_color='FFFFCC', fill_type='solid')}

for row_idx, row_data in enumerate(io_data, start=2):
    for col_idx, value in enumerate(row_data, start=1):
        cell = ws.cell(row=row_idx, column=col_idx)
        cell.value = value
        cell.border = thin_border
        cell.alignment = left_align
        cell.font = normal_font

    # 应用层级颜色
    layer = row_data[2]
    if layer in layer_fills:
        ws.cell(row=row_idx, column=3).fill = layer_fills[layer]

    # 应用方向颜色
    direction = row_data[3]
    if direction in dir_fills:
        ws.cell(row=row_idx, column=4).fill = dir_fills[direction]

# 设置列宽
widths = [12, 25, 8, 10, 20, 20, 15, 8, 10, 10, 12, 12, 18, 18, 30]
for i, width in enumerate(widths, start=1):
    ws.column_dimensions[get_column_letter(i)].width = width

# 冻结窗格
ws.freeze_panes = 'B2'

# 自动筛选
ws.auto_filter.ref = f'A1:O{len(io_data)+1}'

print('IO_Signals sheet completed')

# ==================== Hardware Interfaces工作表 ====================
ws = ws_hw

# 列标题
hw_headers = [
    'Interface ID', 'Interface Name', 'Interface Type', 'Connector Standard',
    'Pin Count', 'Device A', 'Device A Pin', 'Device B', 'Device B Pin',
    'Data Rate', 'Voltage Level', 'Wire Order', 'Physical Location',
    'Shielding Required', 'Max Length (cm)', 'Notes'
]

for col, header in enumerate(hw_headers, start=1):
    cell = ws.cell(row=1, column=col)
    cell.value = header
    cell.font = header_font
    cell.fill = header_fill
    cell.alignment = center_align
    cell.border = thin_border

# 硬件接口数据
hw_data = [
    ['HW-UART-01', 'RPi-STM32 Communication', 'UART', 'JST 3-pin', '3', 'Raspberry Pi 4', 'GPIO14-TX/GPIO15-RX', 'STM32L432KC', 'PA10-RX/PA9-TX', '115200 bps', '3.3V', 'TX-RX-GND', 'Internal PCB', 'Recommended', '15', 'No reverse protection'],
    ['HW-USB-01', 'LiDAR Connection', 'USB', 'USB Micro-B', '4', 'Raspberry Pi 4', 'USB2.0 Port', 'RPLIDAR A2M8', 'USB Micro-B', '480 Mbps', '5V', 'D+/D-/VCC/GND', 'Top-front of vehicle', 'Yes', '50', 'Hot-plug supported'],
    ['HW-USB-02', 'Camera Connection', 'USB', 'USB-C', '4', 'Raspberry Pi 4', 'USB3.0 Port', 'Depth Camera', 'USB-C', '5 Gbps', '5V', 'D+/D-/VCC/GND', 'Top-front of vehicle', 'Yes', '50', 'H.264 encoding'],
    ['HW-I2C-01', 'IMU Communication', 'I2C', 'JST 4-pin', '4', 'STM32L432KC', 'PB8-SCL/PB9-SDA', 'MPU6050 IMU', 'SCL/SDA', '400 kHz', '3.3V', 'SCL-SDA-VCC-GND', 'Vehicle center', 'Recommended', '20', '4.7k pull-up resistors'],
    ['HW-I2C-02', 'Accelerometer Comm', 'I2C', 'JST 4-pin', '4', 'STM32L432KC', 'PB8-SCL/PB9-SDA', 'Adafruit Accel', 'SCL/SDA', '400 kHz', '3.3V', 'SCL-SDA-VCC-GND', 'Vehicle center', 'Recommended', '20', 'Shared I2C bus with IMU'],
    ['HW-GPIO-01', 'Distance Sensor FL', 'Analog', 'JST 3-pin', '3', 'STM32L432KC', 'PA0-ADC', 'SHARP 2Y0A21', 'Vout', 'N/A', '5V', 'Vout-VCC-GND', 'Front-left corner', 'No', '30', 'Analog distance output'],
    ['HW-GPIO-02', 'Distance Sensor FR', 'Analog', 'JST 3-pin', '3', 'STM32L432KC', 'PA1-ADC', 'SHARP 2Y0A21', 'Vout', 'N/A', '5V', 'Vout-VCC-GND', 'Front-right corner', 'No', '30', 'Analog distance output'],
    ['HW-GPIO-03', 'Distance Sensor RL', 'Analog', 'JST 3-pin', '3', 'STM32L432KC', 'PA2-ADC', 'SHARP 2Y0A21', 'Vout', 'N/A', '5V', 'Vout-VCC-GND', 'Rear-left corner', 'No', '30', 'Analog distance output'],
    ['HW-GPIO-04', 'Distance Sensor RR', 'Analog', 'JST 3-pin', '3', 'STM32L432KC', 'PA3-ADC', 'SHARP 2Y0A21', 'Vout', 'N/A', '5V', 'Vout-VCC-GND', 'Rear-right corner', 'No', '30', 'Analog distance output'],
    ['HW-GPIO-05', 'Emergency Stop Button', 'GPIO', 'Push Button', '2', 'Raspberry Pi 4', 'GPIO27', 'E-Stop Module', 'IN/GND', 'N/A', '3.3V', 'IN-GND', 'Top center', 'No', '100', 'Hardware debounce'],
    ['HW-PWM-01', 'Motor Control', 'PWM', 'Motor Wire', '4', 'STM32L432KC', 'PA5-PWM', 'BLDC Motor', 'PWM In', '200 Hz', '12V', 'PWM-PWM-VCC-GND', 'Rear of vehicle', 'Yes', '30', 'Flyback diode protection'],
    ['HW-PWM-02', 'Servo Control', 'PWM', 'Servo Wire', '3', 'STM32L432KC', 'PA6-PWM', 'Steering Servo', 'Signal', '50 Hz', '5V', 'Signal-VCC-GND', 'Front of vehicle', 'No', '25', 'Standard servo 1-2ms pulse'],
    ['HW-POWER-01', 'Battery & Power Dist', 'Power', 'XT60', '2', 'NiMH Battery', '7.2V Output', 'Power Dist Board', '5V/12V Regulators', 'N/A', '7.2V nominal', 'Pos-Neg', 'Bottom center', 'No', '20', 'Voltage sag monitoring']
]

# 填充数据
for row_idx, row_data in enumerate(hw_data, start=2):
    for col_idx, value in enumerate(row_data, start=1):
        cell = ws.cell(row=row_idx, column=col_idx)
        cell.value = value
        cell.border = thin_border
        cell.alignment = left_align
        cell.font = normal_font

# 设置列宽
hw_widths = [14, 25, 14, 18, 10, 18, 22, 18, 22, 14, 14, 20, 20, 16, 16, 30]
for i, width in enumerate(hw_widths, start=1):
    ws.column_dimensions[get_column_letter(i)].width = width

# 冻结窗格
ws.freeze_panes = 'B2'

# 自动筛选
ws.auto_filter.ref = f'A1:P{len(hw_data)+1}'

print('Hardware_Interfaces sheet completed')

# 保存文件
output_file = r'Architecture\CoVAPSy_IO_Bilan_v1.0_2025-11-25.xlsx'
wb.save(output_file)
print(f'File saved: {output_file}')
print(f'Total sheets: {len(wb.sheetnames)}')
print(f'IO Signals: {len(io_data)} records')
print(f'Hardware Interfaces: {len(hw_data)} records')
