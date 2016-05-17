#!/usr/bin/env python

import numpy as np
from scipy.fftpack import fft


def rectify_wave(wave, up_thres=None, low_thres=None):
    for ind, val in enumerate(wave):
        if low_thres is not None and val < low_thres:
            wave[ind] = low_thres
        if up_thres is not None and val > up_thres:
            wave[ind] = up_thres
    return wave


def fourier_reconstruct(original, addition_method=True, num_of_freqs=30):
    if addition_method:
        spectrums, residue = _get_significant_frequencies(original, num_of_freqs*2)
        spectrums = spectrums[0:num_of_freqs]
    else:
        spectrums = _get_highest_n_freq(fft(original), num_of_freqs)
    reconstruction = 0
    for spectrum in spectrums:
        xf = np.linspace(0.0, len(original), len(original))  # frequency varations
        wave = spectrum[0] * np.cos((spectrum[2]*2.0*np.pi*xf) + spectrum[1])
        reconstruction += wave
    return reconstruction, residue


def _get_significant_frequencies(data, total_freq=15, max_addition=10, max_iteration=1000):
    N = len(data)
    xf = np.linspace(0.0, N, N)
    # initialise significant frequencies by taking frequency 0
    spectrum_data = fft(data)
    [amp, phs, freq] = _get_highest_n_freq(spectrum_data, 1)[0]
    frequencies = [[amp, phs, freq]]
    freq_occur_counter = {freq: 1}
    exit_counter = 0
    # data -= amp

    while len(frequencies) < total_freq:
        spectrum_data = fft(data)
        # recreate wave of the highest frequency
        [amp, phs, freq] = _get_highest_n_freq(spectrum_data, 2)[1]
        if freq == 0:
            [amp, phs, freq] = _get_highest_n_freq(spectrum_data, 2)[0]
        wave = amp * np.cos((freq * 2.0 * np.pi * xf) + phs)
        # substracting data with the wave
        data -= wave
        if freq not in zip(*frequencies)[2]:
            frequencies.append([amp, phs, freq])
            freq_occur_counter.update({freq: 1})
        else:
            for ind, val in enumerate(frequencies):
                if frequencies[ind][2] == freq and freq_occur_counter[freq] < max_addition:
                    frequencies[ind][0] += amp
                    frequencies[ind][1] = ((
                        freq_occur_counter[freq] * frequencies[ind][1]
                    ) + phs) / (freq_occur_counter[freq] + 1)
                    freq_occur_counter[freq] += 1
        exit_counter += 1
        if exit_counter >= max_iteration:
            break
    return frequencies, data


def _get_highest_n_freq(freqs, n=15):
    N = len(freqs)
    freqs = freqs[0:N/2]
    indices = [i for i in range(len(freqs))]
    angles = np.angle(freqs)
    amplitudes = np.abs(freqs) / float(N)
    sorted_result = sorted(zip(amplitudes, angles, indices), reverse=True)
    n_freqs = sorted_result[:n]
    return n_freqs
