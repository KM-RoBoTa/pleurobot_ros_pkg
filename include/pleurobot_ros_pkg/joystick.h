#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <stdlib.h>
#ifndef JOYSTICK_H
#define JOYSTICK_H

class Joystick
{
    public:
        double *axes;
        int *buttons;

        Joystick();

        virtual ~Joystick();

        bool load(char*);
        bool is_ready();

        int get_num_buttons();
        int get_num_axes();
        char* get_name();

        bool get_event(js_event*);
        int update(void);

    private:
        int fd;
        bool ready;
        js_event event;
};

#endif // JOYSTICK_H
