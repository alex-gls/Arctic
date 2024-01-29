import os 

file_path = os.path.dirname(os.path.abspath(__file__)) + "/Arctic.urdf"

def main():
    try:
        print(f"Read file: {file_path}")
        with open (file_path, 'r') as f:
            old_data = f.read()
            new_data = old_data.replace('../', 'package://arc_description/')

            # new_data = old_data.replace('<joint name="foot_1"', '<joint name="foot_1_"')
            # new_data = old_data.replace('<joint name="foot_2"', '<joint name="foot_2_"')
            # new_data = old_data.replace('<joint name="foot_3"', '<joint name="foot_3_"')
            # new_data = old_data.replace('<joint name="foot_4"', '<joint name="foot_4_"')
            # new_data = old_data.replace('<joint name="foot_5"', '<joint name="foot_5_"')
            # new_data = old_data.replace('<joint name="foot_6"', '<joint name="foot_6_"')
            # new_data = old_data.replace('<joint name="foot_7"', '<joint name="foot_7_"')
            # new_data = old_data.replace('<joint name="foot_8"', '<joint name="foot_8_"')
            # new_data = old_data.replace('<joint name="foot_9"', '<joint name="foot_9_"')
            # new_data = old_data.replace('<joint name="foot_10"', '<joint name="foot_10_"')
            # new_data = old_data.replace('<joint name="foot_11"', '<joint name="foot_11_"')
            # new_data = old_data.replace('<joint name="foot_12"', '<joint name="foot_12_"')

            # new_data = old_data.replace('<joint name="foot_-3"', '<joint name="foot_-3_"')
            # new_data = old_data.replace('<joint name="foot_-6"', '<joint name="foot_-6_"')
            # new_data = old_data.replace('<joint name="foot_-9"', '<joint name="foot_-9_"')
            # new_data = old_data.replace('<joint name="foot_-12"', '<joint name="foot_-12_"')

        print(new_data)
        print("Replace file")
        with open (file_path, 'w') as f:
            f.write(new_data)

        print("Successfull!")
    except:
        print(Exception)
        
    pass

if __name__ == "__main__":
    main()