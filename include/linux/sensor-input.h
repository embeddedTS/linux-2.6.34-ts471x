#ifndef _SENSOR_INPUT_H
#define _SENSOR_INPUT_H

struct sensor_axis {
	int	x;
	int	y;
	int	z;
};

struct sensor_input_dev {
	struct list_head	list;
	char			*name;
	int			delay;
	//struct completion	thread_exit_complete;
	struct task_struct	*thread_task;
	int 			thread_exit;
	wait_queue_head_t	wait;
	int			on;
	struct  input_dev			*dev;
	int		count;
	void (*report)(struct input_dev	*);
	void (*exit)(struct input_dev	*);
	void (*poweron)(void);
	void (*poweroff)(void);
};

int sensor_input_add(int type, char *name,
	void (*report)(struct input_dev	*),
	void (*exit)(struct input_dev	*),
	void (*poweron)(void),
	void (*poweroff)(void));

void sensor_input_del(char *name);

#define INPUT_G_SENSOR	1
#define INPUT_GYRO_SENSOR	2
#define INPUT_PROXIMITY_SENSOR	3
#define INPUT_AMBIENT_SENSOR	4

#endif /*_SENSOR_INPUT_H*/

