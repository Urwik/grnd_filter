#pragma once
#include <iostream>
#include <ostream>

class color
{
public:
    color(int min = 0, int max = 255){
    this->random(min, max);
    }

    color(int _r, int _g, int _b){
    this->r = _r;
    this->g = _g;
    this->b = _b;
    }

    friend std::ostream& operator<<(std::ostream& os, const color& c)
    {
    os << "[ " << c.r << ", " << c.g << ", " << c.b << " ]" << endl;
    return os;
    }

    void random(int min = 100, int max = 255){
    this->r = (int) min + (rand() % max);
    this->g = (int) min + (rand() % max);
    this->b = (int) min + (rand() % max);

    }

    void normalized(){
    this->r = (float) this->r / 255;
    this->g = (float) this->g / 255;
    this->b = (float) this->b / 255;
    }

    static const color RED_COLOR;
    static const color BLUE_COLOR;
    static const color GREEN_COLOR;
    static const color YELLOW_COLOR;
    static const color WHITE_COLOR;
    static const color ORANGE_COLOR;
    float r;
    float g;
    float b;
};

const color color::RED_COLOR = color(255,0,0);
const color color::BLUE_COLOR = color(0,0,255);
const color color::GREEN_COLOR = color(0,255,0);
const color color::YELLOW_COLOR = color(255,255,0);
const color color::WHITE_COLOR = color(255,255,255);
const color color::ORANGE_COLOR = color(255,165,0);