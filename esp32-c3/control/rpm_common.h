#ifndef RPM_COMMON_H
#define RPM_COMMON_H

// Larger window: more samples = smoother. ESP32-C3 has plenty of RAM.
#define RPM_FILTER_SIZE 24

// IIR coefficient (0.0-1.0). Lower = smoother, higher = more responsive.
#define RPM_IIR_ALPHA 0.35f

class RPMFilter {
  private:
    float buffer[RPM_FILTER_SIZE];
    int index;
    int count;
    float sum;
    float iir_state;

  public:
    RPMFilter() : index(0), count(0), sum(0.0f), iir_state(0.0f) {
        for (int i = 0; i < RPM_FILTER_SIZE; i++) {
            buffer[i] = 0.0f;
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
        float ma = sum / (float)count;
        iir_state = RPM_IIR_ALPHA * ma + (1.0f - RPM_IIR_ALPHA) * iir_state;
        return iir_state;
    }

    void reset() {
        index = 0;
        count = 0;
        sum = 0.0f;
        iir_state = 0.0f;
        for (int i = 0; i < RPM_FILTER_SIZE; i++) {
            buffer[i] = 0.0f;
        }
    }
};

#endif // RPM_COMMON_H
