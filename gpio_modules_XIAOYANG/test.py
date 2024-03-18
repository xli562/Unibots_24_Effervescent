string_input = "U0:50,U1:30, \nU0:51,U1:31,"
inputs = string_input.splitlines()
print(inputs)
for i in inputs:
    # print(string_input.strip())

    readings = i.split(',')
    print(readings)
    for reading in readings:
        if reading.startswith("U"):  # Ultrasonic sensor reading
            ultrasonic_distance = int(reading.split(':')[1])
            sensor_index = int(reading.split(':')[0][1])
            print("sensor_index: {}, distance: {}".format(sensor_index,ultrasonic_distance))
            # self.update_distance(ultrasonic_distance, sensor_index)
            # self.updated_sensors[sensor_index] = True

            # if(self.check_update_status()):
            #     self.updated = True
