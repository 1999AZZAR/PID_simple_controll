#ifndef RPM_COMMON_H
#define RPM_COMMON_H

#define RPM_FILTER_SIZE 10

class RPMFilter {
private:
    float buffer[RPM_FILTER_SIZE];
    int index;
    int count;
    float sum;

public:
    RPMFilter() : index(0), count(0), sum(0.0) {
        for (int i = 0; i < RPM_FILTER_SIZE; i++) {
            buffer[i] = 0.0;
        }
    }

    float update(float newValue) {
        sum -= buffer[index];
        buffer[index] = newValue;
        sum += newValue;
        index = (index + 1) % RPM_FILTER_SIZE;
        
        if (count < RPM_FILTER_SIZE) {
            count++;
        }
        
        return sum / count;
    }

    void reset() {
        index = 0;
        count = 0;
        sum = 0.0;
        for (int i = 0; i < RPM_FILTER_SIZE; i++) {
            buffer[i] = 0.0;
        }
    }
};

#endif // RPM_COMMON_H
