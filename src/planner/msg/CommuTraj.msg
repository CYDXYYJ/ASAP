# the index 
int16 index

# whether intiiator publisher
bool initiator

# duration
float32 duration

# validity
float32 validity

# relative start time
float32 r_start

# trajectory sequence
int32 sequence

# The shape of this agent
float32[] shape

# trajectory in x, defined by 7-th polynomial as 
# a_7 x^7 + a_6 x^6 + ... + a_0
# and  traj_x = [a_7, a_6, ..., a_0]
float32[] traj_x

# trajectory in y
float32[] traj_y

float32[] traj_theta