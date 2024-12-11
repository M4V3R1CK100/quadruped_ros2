
def user_input_loop(node: MyNode):
    global quit
    stand_1 = 0.3
    stand_2 = 1.4

    while quit == 0:
        initial_position(node)
        print("\nMenu:")
        print("1. Stand")
        print("2. Up position")
        print("3. Move (forward/backward)")
        print("4. Translate")
        print("5. Exit")

        
        try:
            imp = input("Enter your choice: ")
            number = int(imp)
            if number == 1:
                print("Setting to Stand position")
                joint_position_state = [stand_1, stand_2, stand_1, stand_2, stand_1, stand_2, stand_1, stand_2]
                node.publish_joint_states(joint_position_state)
            elif number == 2:
                print("Setting to Up position")
                joint_position_state = [-1.0, 2.2, -1.0, 2.2, -1.0, 2.2, -1.0, 2.2]
                node.publish_joint_states(joint_position_state)
                time.sleep(1)
                joint_position_state = [stand_1, stand_2, stand_1, stand_2, stand_1, stand_2, stand_1, stand_2]
                node.publish_joint_states(joint_position_state)
            elif number == 3:
                user = input("Enter Speed (positive for forward, negative for backward): ")
                velocity = int(user)
                movement(node, velocity)  # Llama a la función de movimiento
            elif number == 4:
                print("Translating")
                dummy_traslation(0.1, 0, 0,node.node_joint_states, node.rotation)  # Ajusta según tu función de traducción
            elif number == 5:
                print("Exiting...")
                quit = 1
            else:
                print("Invalid choice, please try again.")
        except ValueError:
            print("Please enter a valid number.")
