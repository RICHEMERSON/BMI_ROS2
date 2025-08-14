from psychopy import visual, core, data, event
import numpy as np
import time
import os
import pickle
import random
import socket  

# Object configuration
object_dict = {}
object_dict['circle'] = {}
object_dict['circle']['Target'] = {}
object_dict['circle']['Target']['size'] = 2.5
object_dict['circle']['Target']['pos'] = [0,0]
object_dict['circle']['Target']['lineColor'] = None  # Remove border
object_dict['circle']['Target']['fillColor'] = (-1,1,-1)  # Green
object_dict['circle']['Target']['fillColorPressed'] = (1,-1,1)  # Purple

object_dict['rectangle'] = {}
object_dict['rectangle']['Target'] = {}
object_dict['rectangle']['Target']['size'] = 2.5
object_dict['rectangle']['Target']['pos'] = [0,0]
object_dict['rectangle']['Target']['lineColor'] = None  
object_dict['rectangle']['Target']['fillColor'] = (-1,1,-1)  
object_dict['rectangle']['Target']['fillColorPressed'] = (1,-1,1)  

object_dict['triangle'] = {}
object_dict['triangle']['Target'] = {}
object_dict['triangle']['Target']['size'] = 2.5
object_dict['triangle']['Target']['pos'] = [0,0]
object_dict['triangle']['Target']['lineColor'] = None  
object_dict['triangle']['Target']['fillColor'] = (-1,1,-1)  
object_dict['triangle']['Target']['fillColorPressed'] = (1,-1,1)  

# Task parameters
SurroundRadius = 10
TouchErr = 4
target_shape = 'triangle'  # Choose: 'circle', 'rectangle', or 'triangle'
use_random_positions = True  # Set to False for fixed center position
ITI = 1.5 # Inter-trial interval in milliseconds
win = visual.Window(size=(1280, 1024), monitor='testMonitor',
                    color=(-1, -1, -1), units='deg', fullscr=True)

# Basic timing objects used for event timestamps
WithinTrailTimer = core.Clock()  
TaskTimer = core.Clock()        

# Minimal UDP setup for publishing event markers (optional)
# Matches the other paradigms so this script can send the same codes if desired.
bhvip = '192.168.137.3' 
bhvport = 8866            
bhvaddr = (bhvip, bhvport) 
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
udp_socket.settimeout(None)  

# Helper to publish event markers over UDP and log a timestamp
def marker_publisher(marker: int, clock=None):  
    """Send an event marker over UDP; optionally record timestamp via provided clock.

    This mirrors the marker mechanism in the center-out paradigms while keeping
    Hold_on mouse-click control unchanged.
    """
    try:
        # Keep encoding consistent with other scripts
        udp_socket.sendto(str(6000 + int(marker)).encode('gbk'), bhvaddr)
    except Exception as e:
        # Non-fatal; continue experiment even if UDP fails
        print(f"[marker_publisher] UDP send failed: {e}")
    ts = clock.getTime() if clock is not None else None
    print(f"Event marker: {marker}, t={ts}")

# Generate 9 grid positions (3x3 grid)
grid_size = 3
screen_width = 20  # in degrees
screen_height = 16  # in degrees
grid_positions = []
for i in range(grid_size):
    for j in range(grid_size):
        x = (i - 1) * (screen_width / 2)  # -10, 0, 10 degrees
        y = (j - 1) * (screen_height / 2)  # -8, 0, 8 degrees
        grid_positions.append((x, y))

# Function to create target based on shape
def create_target(shape, config):
    if shape == 'circle':
        return visual.Circle(win=win, radius=config['size'],
                           lineColor=config['lineColor'], 
                           fillColor=config['fillColor'])
    elif shape == 'rectangle':
        return visual.Rect(win=win, width=config['size']*2, height=config['size']*2,
                          lineColor=config['lineColor'], 
                          fillColor=config['fillColor'])
    elif shape == 'triangle':
        # Create equilateral triangle
        height = config['size'] * np.sqrt(3) # Height of equilateral triangle
        vertices = np.array([[-config['size'], -height/2], 
                           [config['size'], -height/2], 
                           [0, height/2]])
        return visual.ShapeStim(win=win, vertices=vertices,
                              lineColor=config['lineColor'], 
                              fillColor=config['fillColor'])
    else:
        raise ValueError(f"Unknown shape: {shape}")

# # Function to check for quit key
# def check_quit():
#     keys = event.getKeys()
#     if 'q' in keys or 'escape' in keys:
#         return True
#     return False

# Function to get next cursor position (pseudo-random or fixed)
def get_next_cursor_position():
    if use_random_positions:
        return random.choice(grid_positions)
    else:
        return (0, 0)  # Fixed center position

# Stimuli
target = create_target(target_shape, object_dict[target_shape]['Target'])
cursor = visual.Circle(win=win, radius=1,
                       lineColor=(1, 1, 1), fillColor=(1, 1, 1))

# Mouse
mouse = event.Mouse(win=win)

# Trial handler
conds = data.createFactorialTrialList({'reach_dir': np.array([0, np.pi])})
trials = data.TrialHandler(
    trialList=conds,
    nReps=100,
    method='random',
    # [MODIFIED] Align dataTypes with center-out scripts and add timing/marker fields
    dataTypes=[
        'dir', 'click', 'trajectory', 'rt', 'target_shape', 'click_pressed',
        'cursor_position', 'timeout_occurred',
        # Added compatibility fields:
        'EventMarker',            # [ADDED]
        'TrialStartTime',         # [ADDED]
        'TrialError'              # [ADDED]
    ]
)

# Data folder & file
results_folder = 'results'
os.makedirs(results_folder, exist_ok=True)
outfile = os.path.join(results_folder,
    f'center_out_mouse_{target_shape}_{int(time.time())}.pkl')


for t in trials:
    # Book-keeping per trial to mirror center-out paradigms
    TrialEventMarker = []  
    def mark(m):  
        """Append to local marker list and publish via UDP."""
        try:
            TrialEventMarker.append([int(m), WithinTrailTimer.getTime()])
        except Exception:
            # Fallback if clock not available yet
            TrialEventMarker.append([int(m), None])
        marker_publisher(int(m), WithinTrailTimer)
    if quit_experiment:
        break
        
    # Trial start markers and timers (mirrors center-out flow)
    trials.addData('TrialError', 1)                     
    trials.addData('TrialStartTime', TaskTimer.getTime())  
    WithinTrailTimer.reset()                            
    mark(24)                                            

    # Reset - target starts at reach direction, cursor at random grid position
    start_pos = (SurroundRadius * np.cos(t['reach_dir']),
                 SurroundRadius * np.sin(t['reach_dir']))
    
    cursor_pos = get_next_cursor_position()
    cursor.pos = cursor_pos
    target.pos = cursor_pos  # Target follows cursor
    
    # Reset target color to green 
    target.fillColor = object_dict[target_shape]['Target']['fillColor']
    
    trajectory = []
    click = False
    click_pressed = False
    rt_clock = core.Clock()

    # Draw target & cursor
    target.autoDraw = True
    cursor.autoDraw = True
    win.flip()
    mark(1)  # Target on, consistent with other scripts
    
    # # Check for quit during waiting period
    # for i in range(10):  # 1 second wait split into 100ms chunks
    #     core.wait(0.1)
    #     if check_quit():
    #         quit_experiment = True
    #         break
    
    if quit_experiment:
        break

    rt_clock.reset()
    mark(2)  # "Go" equivalent: start monitoring for click
    # Movement phase (max 10s)
    timeout_occurred = False
    while rt_clock.getTime() < 10:
        # read mouse
        x, y = mouse.getPos()
        # check click (mouse cursor reaching target position)
        if np.hypot(x - cursor_pos[0], y - cursor_pos[1]) < TouchErr:
            click = True
            break
            
        # Check for timeout (200ms without click)
        if rt_clock.getTime() > 0.2 and not click_pressed:
            core.wait(ITI-0.2)  # Wait remaining time to complete 200ms
            timeout_occurred = True
            break
        
        # Check for left mouse button press
        mouse_buttons = mouse.getPressed()
        if mouse_buttons[0]:  # Left button pressed
            if not click_pressed:  # First time pressing
                click_pressed = True
                mark(4)  # Click event marker (success/click analogue)
                # Change target color to purple
                target.fillColor = object_dict[target_shape]['Target']['fillColorPressed']
                win.flip()  # Refresh screen after color change
                target.fillColor = object_dict[target_shape]['Target']['fillColor']  # Green
                core.wait(ITI)  # Wait ITI seconds
                win.flip()  # Refresh screen after position update
                trials.addData('TrialError', 0)  # [ADDED] Mark success like center-out
                break  # Exit the movement phase loop to proceed to next trial
        
        # trajectory.append((rt_clock.getTime(), x, y, click_pressed, cursor_pos))
        trajectory.append((rt_clock.getTime(), click_pressed, cursor_pos))
        

        # # check quit
        # if check_quit():
        #     print(trajectory)
        #     quit_experiment = True
        #     break
        
        # Regular flip if no button press
        if not mouse_buttons[0]:
            win.flip()

    # clear stimuli
    target.autoDraw = False
    cursor.autoDraw = False
    win.flip()
    mark(5)  # [ADDED] Trial end/target off marker
    
    # if quit_experiment:
    #     break
        
    core.wait(0.5)

    # record trial
    trials.addData('dir', t['reach_dir'])
    trials.addData('click', int(click))
    trials.addData('trajectory', trajectory)
    trials.addData('rt', rt_clock.getTime())
    trials.addData('target_shape', target_shape)
    trials.addData('click_pressed', click_pressed)
    trials.addData('cursor_position', cursor_pos)
    trials.addData('timeout_occurred', timeout_occurred)
    trials.addData('EventMarker', TrialEventMarker)  # [ADDED]

# save all (even if quit early)
with open(outfile, 'wb') as f:
    pickle.dump(trials.data, f)

# Clean up
win.close()
core.quit()