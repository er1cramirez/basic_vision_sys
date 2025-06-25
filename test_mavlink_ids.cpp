#include <common/mavlink.h>
int main() { printf("ATTITUDE: %d\\n", MAVLINK_MSG_ID_ATTITUDE); printf("ATTITUDE_QUATERNION: %d\\n", MAVLINK_MSG_ID_ATTITUDE_QUATERNION); return 0; }
