#ifndef RPM_COMMON_H
#define RPM_COMMON_H

// Sliding window filter for more robust RPM smoothing
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
        // Remove oldest value from sum
        sum -= buffer[index];
        
        // Add new value
        buffer[index] = newValue;
        sum += newValue;
        
        // Move to next index
        index = (index + 1) % RPM_FILTER_SIZE;
        
        // Track how many samples we have
        if (count < RPM_FILTER_SIZE) {
            count++;
        }
        
        // Return average
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
