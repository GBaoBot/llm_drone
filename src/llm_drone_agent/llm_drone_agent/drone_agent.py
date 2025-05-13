from rosa import ROSA

class DroneAgent(ROSA):
    def __init__(self, streaming: bool = False, verbose: bool = True):
        
        move_forward_for_duration = Tool(
            name='move_forward_for_duration',
            func=self.move_forward_for_duration,
            description='Move the drone forward for a specified duration.',
        )
        
        super().__init__(
            ros_version=1,
            llm=self.__llm,
            tools=[cool_turtle_tool, blast_off],
            tool_packages=[turtle_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            streaming=streaming,
        )