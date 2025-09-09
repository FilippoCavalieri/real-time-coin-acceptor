import serial
import numpy
import matplotlib.pyplot as plt
from scipy import stats  # Needed for confidence interval calculations

MAX_ITER = 15


def plot_histograms(weights, times):
    # Flatten weights for global analysis
    flat_weights = [w for sublist in weights for w in sublist]

    # --- Histogram of Weights ---
    plt.figure()
    plt.hist(flat_weights, bins=[x for x in range(80, 520, 10)], edgecolor='black')
    plt.title('Histogram of Weights')
    plt.xlabel('Weight')
    plt.ylabel('Frequency')
    plt.grid(True)

    # --- Histogram of Times ---
    plt.figure()
    plt.hist(times, bins=[x for x in range(-100, 201, 10)], edgecolor='black')
    plt.title('Histogram of Times')
    plt.xlabel('Time')
    plt.ylabel('Frequency')
    plt.grid(True)

    # --- Line chart: 15 temporal series of weights ---
    plt.figure()
    for i, w_series in enumerate(weights):
        if w_series:
            plt.plot(w_series, marker='o', label=f'Series {i+1}')

    plt.title('Temporal Series of Weights (15 Iterations)')
    plt.xlabel('Sample Index (within iteration)')
    plt.ylabel('Weight')
    plt.ylim(top=300)  # Imposta il limite massimo dell'asse Y
    plt.legend(loc='upper right', fontsize='small', ncol=2)
    plt.grid(True)

    # --- Percentile-based 95% CI for all weights ---
    lower_bound = numpy.percentile(flat_weights, 2.5)
    upper_bound = numpy.percentile(flat_weights, 97.5)
    weight_mean = numpy.mean(flat_weights)

    plt.axhspan(lower_bound, upper_bound, color='green', alpha=0.1, label='95% CI (All Data)')
    plt.axhline(weight_mean, color='green', linestyle='--', linewidth=1, label='Mean Weight')

    # --- Time Histogram with 95% CI (filtered) ---
    filtered_times = [t for t in times if t >= 50]

    if len(filtered_times) > 1:
        t_mean = numpy.mean(filtered_times)
        t_sem = stats.sem(filtered_times)
        z_95 = stats.norm.ppf(0.975)
        t_ci = z_95 * t_sem
        t_lower = t_mean - t_ci
        t_upper = t_mean + t_ci
    else:
        t_mean = filtered_times[0] if filtered_times else 0
        t_lower = t_upper = t_mean

    plt.figure()
    plt.hist(filtered_times, bins=[x for x in range(50, 201, 10)], edgecolor='black', alpha=0.7)
    plt.axvline(t_mean, color='red', label='Mean Time')
    plt.axvspan(t_lower, t_upper, color='red', alpha=0.3, label='95% CI')
    plt.title('Time Histogram (Filtered) with 95% Confidence Interval')
    plt.xlabel('Time')
    plt.ylabel('Frequency')
    plt.legend()
    plt.grid(True)

    plt.show()




def main():
    print("Inizio calibrazione")
    ser = serial.Serial('/dev/ttyACM0')
    weights = [[] for _ in range(MAX_ITER)]
    times = [0 for _ in range(MAX_ITER)]

    for i in range(MAX_ITER):
        while True:
            packet = str(ser.readline().decode('utf-8')).strip()
            print(f'Pacchetto ricevuto: "{packet}"')

            if not packet:
                continue

            pType = packet[0]
            try:
                pValue = int(packet[1:])
            except ValueError:
                print("Errore: valore non numerico")
                continue

            if pType == 'w':
                weights[i].append(pValue)
            elif pType == 't':
                times[i] = pValue
                break

    plot_histograms(weights, times)

if __name__ == "__main__":
    main()
