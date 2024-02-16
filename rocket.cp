//irem karakaplan
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>

const float g = 9.81f;

using namespace std;
using namespace std::chrono;

class Velocity {
private:
    double velocityVal;

public:
    Velocity() : velocityVal(0.0) {}

    double getVelocityVal() const {
        return velocityVal;
    }

    void setVelocityVal(double velocityVal) {
        this->velocityVal = velocityVal;
    }

    double updateVelocity(double vx, double vy) {
        return this->velocityVal = sqrt(pow(vx, 2) + pow(vy, 2));
    }

    double getVelocityX(double angle) const {
        return this->velocityVal * cos(angle);
    }

    double getVelocityY(double angle) const {
        return this->velocityVal * sin(angle);
    }
};

class RocketMotorStatus {
private:
    bool status;

public:
    RocketMotorStatus() : status(true) {}

    bool getStatus() const {
        return status;
    }

    void setStatus(bool status) {
        this->status = status;
    }
};

class ParachuteStatus {
private:
    bool status;

public:
    ParachuteStatus() : status(false) {}

    bool getStatus() const {
        return status;
    }

    void setStatus(bool status) {
        this->status = status;
    }
};

class Angle {
private:
    double angleValue;

public:
    Angle() : angleValue(0.0) {}

    Angle(double angleValue) : angleValue(angleValue) {}

    double getAngleValue() const {
        return angleValue;
    }

    void setAngleValue(double angleValue) {
        this->angleValue = angleValue;
    }

    double toRadians() const {
        return this->angleValue * M_PI / 180;
    }
};

class Altitude {
private:
    double altitudeValue;

public:
    Altitude() : altitudeValue(0.0) {}

    double calculateAltitude(double vy, double t) const {
        return vy * t - 0.5 * g * t * t;
    }

    double getAltitudeValue() const {
        return altitudeValue;
    }

    void setAltitudeValue(double altitudeValue) {
        this->altitudeValue = altitudeValue;
    }
};

class Rocket {
private:
    Velocity velocity;
    RocketMotorStatus rocketMotorStatus;
    ParachuteStatus parachuteStatus;
    Angle angle;
    Altitude altitude;
    double mass;
    double burnoutTime;

public:
    Rocket() : velocity(), rocketMotorStatus(), angle(60), parachuteStatus(), altitude(), mass(17.810), burnoutTime(10) {}

    void launch(double duration) {
        rocketMotorStatus.setStatus(true);
    }

    double calculateDragForce(double velocity) const {
        // take the cross Sectional Area=0.2 m^2
        // take the drag coefficient=0.4
        return 0.5 * 1.293 * 0.4 * 0.2 * velocity*velocity ;
    }

    void updateStatus() {
        ofstream csvFile("altitude_output.csv");
        ofstream csvFile2("velocity_output.csv");
        double durationSeconds = 50;
        double thrustForce = 400;
        double angleRadian = angle.toRadians();
        double terminal_velocity=sqrt((2*mass*g)/(0.5 * 1.293 * 0.4 * 0.2));


        double vx = velocity.getVelocityX(angleRadian);
        double vy = velocity.getVelocityY(angleRadian);
        double t = 0.0;

        double ax_thrust = thrustForce * cos(angleRadian) / mass;
        double ay_thrust = thrustForce * sin(angleRadian) / mass;

        // set time step




        while (t<durationSeconds) {
            // Euler method
            if(t==burnoutTime){
                ax_thrust=0;
                ay_thrust=0;
            }
            double dragForceX = calculateDragForce(vx);
            double dragForceY = calculateDragForce(vy);



            // set acceleration

            double ax = ax_thrust - (dragForceX / mass);
            if(ax<=0)
                ax=0;

            double ay = ay_thrust - g;

            vx += ax ;
            if(vy<terminal_velocity) {
                vy += ay;
            }
            else{
                vy=terminal_velocity;
            }
            velocity.updateVelocity(vx, vy);

            // update location
            double locationX = vx * t;

            double locationY = (vy * t - 0.5 * g * t * t);
            if(ay<=0)
                parachuteStatus.setStatus(true);
            if(locationY<=0)
                locationY=0;
            double altitudeVal = locationY;
            altitude.setAltitudeValue(altitudeVal);

            if (t == burnoutTime) {
                rocketMotorStatus.setStatus(false);
            }

            // Display information
            displayInfo(t, locationX);
            csvFile<<t<<","<<altitude.getAltitudeValue()<<endl;
            csvFile2<<t<<","<<velocity.getVelocityVal()<<endl;
            if(locationY==0&&t!=0){
                cout<<"The rocket is at the surface level";
                break;
            }
            t++;
            this_thread::sleep_for(seconds(1));
        }
    }






    void displayInfo(double time, double locationX) const {
        cout << "Elapsed time:" << time <<" seconds"<< endl;
        cout << "Velocity: " << velocity.getVelocityVal() << endl;
        cout << "rocketMotorStatus: " << rocketMotorStatus.getStatus() << endl;
        cout << "parachuteStatus:"  <<parachuteStatus.getStatus()<< endl;
        cout << "Altitude(Y location): " << altitude.getAltitudeValue() << endl;
        cout << "X location:" << locationX << endl;

    }
};

int main() {
    Rocket myRocket;
    myRocket.updateStatus();

}
