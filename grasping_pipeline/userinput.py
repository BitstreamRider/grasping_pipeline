import yasmin
import rclpy
import rclpy
from std_msgs.msg import String



class UserInput(yasmin.State):
    '''
    A state that waits for user input and returns the corresponding outcome.
    
    Prints a menu with the possible outcomes and waits for the user to enter a key. Depending on the
    key, the corresponding outcome is returned.
    
    Returns
    -------
    The outcome corresponding to the key entered by the user.
    '''

    def __init__(self, node, key_outcome_description):
        '''
        Initializes the UserInput state. The key_outcome_description dictionary maps pressed keys to outcomes.
        
        Parameters
        ----------
        key_outcome_description: dict
            A dictionary that maps keys to outcomes. The keys should be single characters. The values should
            be lists with two entries. The first entry should be the outcome that is returned when the key is
            pressed. The second entry should be a description of the key that is printed in the menu.
        
        Example
        -------
        key_outcome_description = {'c': ['success', 'continue'], 'r': ['abort', 'reset state']}
        
        This dictionary maps the 'c' key to the 'success' outcome and prints 'continue' as a description
        in the menu. The 'r' key is mapped to the 'abort' outcome and 'reset state' is printed as a description.
        '''
        self.node = node
        self.map = key_outcome_description
        self.data =  None
        self.subscription = self.node.create_subscription(String, 'user_input', self.listener_callback, 10)
        
        outcomes = []
        descriptions = []
        keys = []

        for key in key_outcome_description:

            if len(key) != 1:
                self.node.get_logger().error(
                    'Error while seting up UserInput: Key should only be a single character!')
                self.node.get_logger().error('Entry that led to error: ' +
                                             str(key) + ' : ' + str(key_outcome_description[key]))

            if len(key_outcome_description[key]) != 2:
                self.node.get_logger().error(
                    'Error while setting up UserInput: Value should have exactly two entries!')
                self.node.get_logger().error('Entry that led to error: ' +
                                             str(key) + ' : ' + str(key_outcome_description[key]))

            outcomes.append(key_outcome_description[key][0])

        super().__init__(outcomes)

    def listener_callback(self, msg):
        self.data = msg.data

    def execute(self, blackboard: yasmin.Blackboard):
        '''Prints the menu and waits for user input. Returns the outcome corresponding to the key pressed.

        Returns
        -------
        str
            The outcome corresponding to the key pressed by the user.
        '''
        self.node.get_logger().info('Executing state UserInput')
        self.print_menu()
        return self.handle_userinput()

    def print_menu(self):
        '''Prints the menu with the possible outcomes.'''
        print('Enter command:')
        for key in self.map:
            print('\t' + str(key) + ' - ' + str(self.map[key][1]))

    def handle_userinput(self):
        '''Waits for user input and returns the corresponding outcome.

        Returns
        -------
        str
            The outcome corresponding to the key pressed by the user.
        '''
        self.node.get_logger().info("Waiting for user_input publisher...")
        while not self.data:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        while rclpy.ok():
            user_input = self.data
            self.data = None
            if len(user_input) != 1:
                print('Please enter only one character')
                continue
            char_input = user_input.lower()
            if char_input not in self.map:
                print('Invalid key!')
                continue
            return self.map[char_input][0]


