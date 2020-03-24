using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ConsoleApp1
{
    public static class Filters
    {
        public const double SinglePi = Math.PI;
        public const double DoublePi = 2 * Math.PI;

        public static Dictionary<double, double> GetJoinedSpectrum(
            IList<Complex2> spectrum0, IList<Complex2> spectrum1,
            double shiftsPerFrame, double sampleRate)
        {
            var frameSize = spectrum0.Count;
            var frameTime = frameSize / sampleRate;
            var shiftTime = frameTime / shiftsPerFrame;
            var binToFrequancy = sampleRate / frameSize;
            var dictionary = new Dictionary<double, double>();

            for (var bin = 0; bin < frameSize; bin++)
            {
                var omegaExpected = DoublePi * (bin * binToFrequancy); // ω=2πf
                var omegaActual = (spectrum1[bin].phase - spectrum0[bin].phase) / shiftTime; // ω=∂φ/∂t
                var omegaDelta = Align(omegaActual - omegaExpected, DoublePi); // Δω=(∂ω + π)%2π - π
                var binDelta = omegaDelta / (DoublePi * binToFrequancy);
                var frequancyActual = (bin + binDelta) * binToFrequancy;
                var magnitude = spectrum1[bin].magnitude + spectrum0[bin].magnitude;
                dictionary.Add(frequancyActual, magnitude * (0.5 + Math.Abs(binDelta)));
            }

            return dictionary;
        }

        public static double Align(double angle, double period)
        {
            var qpd = (int)(angle / period);
            if (qpd >= 0) qpd += qpd & 1;
            else qpd -= qpd & 1;
            angle -= period * qpd;
            return angle;
        }
    }
}
