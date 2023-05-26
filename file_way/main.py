import threading, time, stuff

def doing_something():
    """Wrapper function for doing work. This is like the 'robot_run' function."""
    stuff.do_work()
    
def shutdown():
    """Shutdown function. Calls flags to end process."""
    stuff.running = False
    stuff.flag = False
    
def ui():
    """
    UI (user interface) function to get commands from user. This runs in the 
    main thread
    """
    val = input('cmd ')
    if val == 'start':
        stuff.flag = True
    elif val == 'work':
        print(stuff.work)
    elif val == 'stop':
        stuff.flag = False
    elif val == 'q':
        shutdown()
    else:
        print('unknown command')
        
    return val


if __name__ == '__main__':
    # initialize the work thread and start it
    work_thread = threading.Thread(target=doing_something)
    work_thread.start()

    # Try except statement to catch exceptions that happen in the ui function.
    try:
        while stuff.running:
            ui()
    except:
        shutdown()
        
    print('Closing threads cleanly')
    work_thread.join()