#include "pleurobot_ros_pkg/joystick.h"

Joystick::Joystick()
{
    this->ready = false;
//    buttons=(int*) calloc (Joystick::get_num_buttons(), sizeof(int));
//    axes=(double*) calloc (Joystick::get_num_axes(), sizeof(double));
    buttons=(int*) calloc (22, sizeof(int));
    axes=(double*) calloc (22, sizeof(double));
}

Joystick::~Joystick()
{
    //dtor
    free(buttons);
    free(axes);
}

bool Joystick::load(char* device)
{
    this->fd = open(device, O_RDONLY | O_NONBLOCK);
    if(this->fd > 0)
    {
        this->ready = true;
    }
    else
    {
        this->ready = false;
    }

    return this->ready;
}

bool Joystick::is_ready()
{
    return this->ready;
}

int Joystick::get_num_buttons()
{
    char number_of_buttons;
    ioctl(this->fd, JSIOCGBUTTONS, &number_of_buttons);
    return (int)number_of_buttons;
}

int Joystick::get_num_axes()
{
    char number_of_axes;
    ioctl(this->fd, JSIOCGAXES, &number_of_axes);
    return (int)number_of_axes;
}

char* Joystick::get_name()
{
    char* name = new char[128];
    ioctl(this->fd, JSIOCGNAME(128), name);
    return name;
}

bool Joystick::get_event(js_event* event)
{
    int bytes = read(this->fd, event, sizeof(*event));

    if(bytes != -1 && bytes == sizeof(*event))
    {
        return true;
    }

    return false;
}

int Joystick::update(void)
{
    if(Joystick::get_event(&event)){

        if((event.type == JS_EVENT_BUTTON) && (event.type != JS_EVENT_AXIS)){
            buttons[event.number] = event.value;
        }
        else if(event.type == JS_EVENT_AXIS)
        {
            axes[event.number]=event.value>0?event.value/32767.:event.value/32767.;
        }
    }
    return 1;
}




