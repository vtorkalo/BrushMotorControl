﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ChartPlot
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            using (Graphics g = pictureBox1.CreateGraphics()) // Use the CreateGraphics method to create a graphic and draw on the picture box. Use using in order to free the graphics resources.
            {
                for (int i=0; i< data.Count-1; i++)
                { 
                    int x1 = i-300;
                    int y1 = 1500 - data[i]/2;
                    int x2 = i+1-300;
                    int y2 =1500 - data[i+1]/2;
                    g.DrawLine(new Pen(Color.Black), x1*2f, y1, x2*2f, y2);
                }
            }
        }
        List<int> data = new List<int>
    {
      1934, 1929, 1935, 1929, 1927, 1937, 1927, 1937, 1925, 1935, 1927, 1939, 1938, 1940, 1938, 1938, 1932, 1937, 1941, 1938, 1932, 1933, 1936, 1934, 1938, 1935, 1937, 1938, 1931, 1944, 1936, 1941, 1935, 1941, 1935, 1942, 1939, 1936, 1937, 1932, 1934, 1940, 1943, 1933, 1939, 1935, 1943, 1939, 1939, 1945, 1940, 1937, 1944, 1934, 1943, 1935, 1939, 1941, 1933, 1945, 1938, 1939, 1937, 1939, 1939, 1933, 1931, 1945, 1939, 1937, 1943, 1937, 1945, 1940, 1948, 1942, 1945, 1941, 1939, 1941, 1938, 1939, 1929, 1932, 1947, 1937, 1945, 1935, 1942, 1941, 1937, 1946, 1933, 1949, 1942, 1939, 1943, 1936, 1943, 1946, 1942, 1946, 1947, 1954, 1948, 1946, 1945, 1957, 1966, 1967, 1965, 1970, 1970, 1979, 1975, 1980, 1974, 1979, 1972, 1981, 1978, 1991, 1982, 1980, 1983, 1267, 1301, 1310, 1308, 1318, 1268, 1253, 1239, 1235, 1232, 1247, 1242, 1250, 1244, 1251, 1245, 1215, 1158, 1099, 1072, 1069, 1078, 1086, 1120, 1153, 1201, 1219, 1239, 1267, 1266, 1281, 1279, 1282, 1273, 1271, 1258, 1257, 1251, 1230, 1243, 1257, 1253, 1227, 1209, 1206, 1209, 1224, 1228, 1247, 1274, 1311, 1339, 1373, 1390, 1425, 1437, 1445, 1453, 1475, 1483, 1492, 1483, 1481, 1479, 1479, 1489, 1503, 1495, 1491, 1489, 1493, 1488, 1506, 1519, 1534, 1537, 1561, 1582, 1601, 1627, 1641, 1665, 1673, 1975, 1967, 1971, 1989, 2008, 2006, 2003, 1995, 1997, 1989, 2002, 1961, 1998, 1993, 2003, 1999, 1994, 2006, 2001, 2004, 2000, 1999, 2003, 1999, 2003, 2002, 1999, 2002, 1995, 1997, 1998, 1992, 1999, 1991, 1997, 2000, 1999, 1996, 1999, 1995, 1993, 1996, 1991, 1995, 1997, 1991, 1998, 1991, 1994, 1986, 1987, 1997, 1991, 1999, 1995, 1991, 2002, 1994, 1983, 1986, 1982, 1989, 1983, 1987, 1990, 1987, 1986, 1986, 1974, 1991, 1975, 1987, 1974, 1970, 1971, 1967, 1971, 1969, 1966, 1964, 1970, 1962, 1966, 1959, 1962, 1965, 1969, 1967, 1967, 1973, 1967, 1969, 1962, 1967, 1967, 1971, 1953, 1971, 1965, 1965, 1965, 1964, 1960, 1966, 1961, 1958, 1954, 1951, 1955, 1951, 1954, 1955, 1947, 1950, 1953, 1954, 1946, 1953, 1949, 1949, 1946, 1944, 1951, 1945, 1949, 1950, 1951, 1945, 1943, 1948, 1951, 1948, 1953, 1946, 1941, 1949, 1947, 1938, 1951, 1946, 1959, 1941, 1946, 1940, 1943, 1944, 1945, 1941, 1945, 1937, 1947, 1942, 1942, 1935, 1947, 1943, 1946, 1939, 1949, 1937, 1943, 1944, 1939, 1941, 1944, 1935, 1950, 1945, 1939, 1948, 1940, 1943, 1939, 1940, 1945, 1947, 1939, 1943, 1942, 1940, 1941, 1943, 1943, 1939, 2199, 2253, 2281, 2332, 2359, 2392, 2399, 2426, 2430, 2446, 2449, 2462, 2519, 2583, 2622, 2647, 2678, 2667, 2681, 2663, 2647, 2609, 2605, 2581, 2562, 2576, 2598, 2621, 2625, 2651, 2664, 2668, 2669, 2581, 2564, 2561, 2566, 2583, 2558, 2556, 2563, 2561, 2551, 2548, 2560, 2562, 2561, 2542, 2527, 2513, 2504, 2481, 2466, 2439, 2413, 2390, 2372, 2355, 2325, 2326, 2311, 2298, 2287, 2279, 1991, 1956, 1943, 1919, 1920, 1905, 1910, 1924, 1919, 1917, 1935, 1924, 1915, 1880, 1877, 1876, 1882, 1877, 1883, 1883, 1883, 1867, 1887, 1879, 1878, 1875, 1879, 1883, 1882, 1892, 1897, 1900, 1889, 1884, 1887, 1887, 1882, 1883, 1881, 1890, 1887, 1886, 1893, 1885, 1890, 1890, 1889, 1886, 1893, 1891, 1890, 1899, 1890, 1892, 1882, 1891, 1888, 1915, 1914, 1916, 1913, 1918, 1916, 1910, 1915, 1914, 1919, 1915, 1917, 1910, 1924, 1912, 1920, 1919, 1928, 1918, 1930, 1922, 1928, 1923, 1929, 1928, 1926, 1924, 1932, 1932, 1931, 1927, 1932, 1934, 1931, 1935, 1935, 1935, 1934, 1936, 1927, 1930, 1931, 1926, 1934, 1937, 1935, 1931, 1927, 1939, 1933, 1939, 1940, 1939, 1939, 1937, 1945, 1943, 1939, 1942, 1933, 1938, 1947, 1940, 1946, 1940, 1943, 1939, 1937, 1939, 1939, 1943, 1934, 1939, 1938, 1975, 1981, 1991, 1976, 1983, 1990, 1980, 1971, 1983, 1983, 1978, 1975, 1977, 1976, 1967, 1974, 1971, 1897, 1816, 1769, 1729, 1704, 1676, 1076, 1080, 1066, 1064, 1005, 1040, 1025, 1018, 1003, 999, 996, 996, 1012, 985, 947, 926, 929, 941, 937, 939, 1019, 1059, 1574, 1580, 1578, 1576, 1584, 1603, 1590, 1605, 1605, 1617, 1647, 1670, 1685, 1702, 1725, 1743, 1754, 1766, 1782, 1794, 1789, 2003, 1999, 2002, 1999, 2003, 2003, 1996, 2002, 1992, 2004, 1996, 1998, 2003, 1994, 1995, 1990, 1999, 1991, 1996, 1994, 1988, 1994, 1999, 1996, 1991, 1991, 1999, 1987, 1994, 1994, 1986, 1979, 1995, 1999, 1997, 1987, 1999, 1985, 1991, 1993, 1987, 1958, 1963, 1971, 1965, 1962, 1961, 1963, 1958, 1962, 1952, 1958, 1962, 1954, 1957, 1957, 1962, 1951, 1956, 1949, 1951, 1948, 1959, 1948, 1957, 1954, 1948, 1957, 1948, 1947, 1953, 1947, 1943, 1943, 1947, 1948, 1951, 1952, 1951, 1941, 1945, 1943, 1941, 1941, 1943, 1938, 1946, 1952, 1945, 1942, 1940, 1941, 1944, 1947, 1940, 1939, 1942, 1946, 1932, 1925, 1922, 1919, 1918, 1924, 1905, 1913, 1909, 1901, 1905, 1898, 1904, 1899, 1897, 1906, 1903, 1896, 2759, 2771, 2781, 2763, 2716, 2683, 2633, 2567, 2571, 2578, 2604, 2631, 2646, 2659, 2662, 2681, 2695, 2527, 2501, 2465, 2452, 2434, 2416, 2397, 2392, 2385, 2372, 2357, 2367, 2361, 2359, 2369, 2367, 2352, 1975, 1960, 1941, 1931, 1919, 1900, 1899, 1902, 1911, 1920, 1914, 1914, 1907, 1877, 1877, 1874, 1883, 1895, 1883, 1887, 1882, 1892, 1889, 1896, 1883, 1897, 1887, 1890, 1887, 1891, 1893, 1887, 1892, 1884, 1895, 1895, 1899, 1897, 1897, 1903, 1900, 1907, 1908, 1911, 1903, 1918, 1912, 1905, 1908, 1914, 1924, 1929, 1930, 1928, 1927, 1927, 1923, 1920, 1925, 1919, 1928, 1924, 1921, 1924, 1917, 1923, 1928, 1929, 1936, 1934, 1935, 1935, 1937, 1935, 1934, 1938, 1928, 1939, 1933, 1940, 1940, 1938, 1939, 1940, 1944, 1943, 1939, 1935, 1941, 1943, 1945, 1929, 1943, 1943, 1942, 1935, 1979, 1981, 1977, 1980, 1975, 1983, 1971, 1971, 1977, 1971, 1963, 1927, 1885, 1857, 1845, 1802, 1006, 986, 989, 982, 973, 926, 894, 878, 869, 872, 888, 891, 933, 999, 1546, 1546, 1546, 1540, 1531, 1534, 1538, 1545, 1543, 1553, 1567, 1564, 1583, 1583, 1595, 1995, 1995, 2000, 2019, 1999, 1998, 1996, 1998, 2009, 1997, 1997, 1991, 2002, 2002, 1990, 1996, 1997, 1994, 1996, 1992, 1987, 1997, 1988, 1996, 1988, 1998, 1993, 1995, 1983, 1985, 1982, 1985, 1977, 1973, 1974, 1975, 1977, 1971, 1966, 1961, 1967, 1960, 1958, 1967, 1955, 1961, 1959, 1960, 1959, 1964, 1956, 1956, 1962, 1955, 1957, 1963, 1947, 1943, 1945, 1944, 1946, 1951, 1943, 1948, 1947, 1939, 1957, 1937, 1955, 1947, 1940, 1932, 1947, 1941, 1941, 1938, 1938, 1938, 1947, 1935, 1943, 1946, 1947, 1902, 1915, 1903, 1909, 1903, 1905, 1909, 1909, 1905, 1903, 1907, 1900, 1899, 2718, 2683, 2716, 2704, 2707, 2727, 2768, 2831, 2874, 2895, 2902, 2900, 2884, 2443, 2435, 2427, 2405, 2403, 2418, 2404, 2417, 2424, 2416, 2411, 2428, 2428, 2036, 2023, 1999, 1979, 1953, 1943, 1926, 1922, 1912, 1910, 1921, 1932, 1927, 1890, 1879, 1887, 1887, 1876, 1885, 1885, 1879, 1889, 1883, 1886, 1879, 1886, 1891, 1891, 1899, 1891, 1895, 1895, 1900, 1893, 1896, 1893, 1892, 1899, 1924, 1935, 1919, 1930, 1921, 1917, 1932, 1923, 1929, 1922, 1932, 1927, 1931, 1935, 1938, 1926, 1934, 1933, 1935, 1934, 1927, 1935, 1934, 1942, 1940, 1935, 1937, 1944, 1939, 1941, 1938, 1940, 1945, 1938, 1936, 1942, 1934, 1943, 1936, 1937, 1945, 1941, 1937, 1947, 1939, 1950, 1947, 1947, 1327, 1321, 1319, 1320, 1287, 1242, 1187, 1157, 1147, 1147, 1141, 1151, 1224, 1216, 1223, 1211, 1189, 1165, 1170, 1165, 1165, 1175, 1203, 1234, 1675, 1672, 1677, 1690, 1699, 1702, 1707, 1707, 1714, 1716, 1729, 1999, 2005, 2001, 2002, 2004, 1994, 2005, 1999, 2003, 1995, 2004, 2003, 1994, 1999, 1997, 1994, 1999, 1994, 1990, 1995, 2002, 1996, 1983, 1969, 1965, 1966, 1959, 1958, 1962, 1971, 1961, 1952, 1961, 1960, 1964, 1966, 1975, 1969, 1957, 1971, 1961, 1964, 1970, 1971, 1966, 1963, 1947, 1949, 1950, 1946, 1941, 1941, 1943, 1944, 1943, 1939, 1945, 1951, 1946, 1949, 1947, 1947, 1942, 1943, 1943, 1944, 1939, 1915, 1906, 1905, 1896, 1901, 1908, 1901, 1903, 1910, 1900, 1897, 2674, 2669, 2635, 2601, 2651, 2545, 2524, 2537, 2557, 2588, 2597, 2639, 2597, 2572, 2549, 2528, 2501, 2478, 2463, 2448, 2443, 2419, 2107, 2106, 2106, 2096, 2078, 2067, 2056, 2025, 2012, 2002, 1882, 1889, 1891, 1889, 1884, 1887, 1886, 1893, 1886, 1887, 1891, 1890, 1891, 1893, 1883, 1891, 1884, 1887, 1890, 1885, 1891, 1919, 1910, 1917, 1911, 1913, 1920, 1919, 1915, 1921, 1921, 1923, 1921, 1925, 1922, 1926, 1915, 1927, 1924, 1927, 1930, 1927, 1929, 1939, 1937, 1930, 1943, 1934, 1941, 1942, 1943, 1939, 1940, 1942, 1939, 1939, 1939, 1938, 1941, 1943, 1935, 1942, 1979, 1975, 1931, 1895, 1855, 1772, 1673, 1594, 1535, 1478, 1223, 1205, 1217, 1222, 1226, 1208, 1193, 1192, 1143, 1084, 1433, 1434, 1450, 1452, 1463, 1449, 1452, 1449, 1448, 1461, 1832, 1853, 1869, 1884, 1903, 1909, 1922, 1922, 1935, 1995, 1990, 1985, 1993, 1998, 1996, 1995, 1991, 1992, 1990, 1987, 1985, 1996, 1988, 1994, 1995, 1993, 1991, 1994, 1983, 1959, 1970, 1972, 1969, 1965, 1969, 1966, 1963, 1966, 1966, 1967, 1965, 1956, 1964, 1951, 1954, 1955, 1954, 1955, 1941, 1953, 1945, 1945, 1943, 1943, 1944, 1944, 1943, 1943, 1941, 1936, 1936, 1943, 1939, 1946, 1942, 1944, 1936, 1909, 1903, 1905, 1905, 1924, 1909, 1911, 1953, 2005, 2659, 2608, 2595, 2563, 2551, 2566, 2578, 2608, 2611, 2524, 2503, 2495, 2491, 2487, 2503, 2498, 2490, 2500, 2488, 2048, 2033, 2026, 1995, 1987, 1965, 1961, 2018, 1973, 1883, 1889, 1890, 1886, 1879, 1893, 1891, 1886, 1891, 1893, 1890, 1890, 1889, 1887, 1895, 1887, 1892, 1879, 1908, 1903, 1915, 1918, 1912, 1919, 1920, 1926, 1915, 1926, 1925, 1925, 1921, 1924, 1918, 1930, 1926, 1923, 1941, 1932, 1938, 1930, 1940, 1942, 1940, 1931, 1937, 1942, 1942, 1939, 1942, 1935, 1935, 1941, 1938, 1944, 1982, 1983, 1978, 1978, 1974, 1983, 1978, 1973, 904, 892, 898, 916, 951, 989, 1024, 1048, 1074, 1378, 1365, 1363, 1364, 1367, 1378, 1388, 1390, 1370, 1787, 1788, 1794, 1803, 1827, 1851, 1854, 1865, 1999, 1999, 1993, 2004, 2007, 2002, 1996, 1991, 1997, 1991, 1989, 1991, 1993, 1995, 1994, 1990, 1988, 1986, 1963, 1963, 1966, 1965, 1967, 1965, 1963, 1971, 1953, 1957, 1951, 1958, 1959, 1964, 1965, 1961, 1961, 1944, 1952, 1940, 1947, 1959, 1934, 1943, 1937, 1940, 1947, 1945, 1944, 1945, 1941, 1942, 1936, 1921, 1913, 1909, 1914, 1908, 1904, 1908, 1907, 1899, 2603, 2629, 2642, 2658, 2666, 2674, 2677, 2666, 2748, 2739, 2739, 2712, 2689, 2651, 2612, 2575, 2125, 2116, 2048, 2115, 2116, 2119, 2118, 2115, 1882, 1878, 1886, 1889, 1878, 1890, 1877, 1890, 1883, 1887, 1884, 1878, 1884, 1875, 1893, 1884, 1889, 1911, 1907, 1907, 1908, 1915, 1908, 1910, 1910, 1927, 1933, 1927, 1926, 1932, 1923, 1923, 1930, 1935, 1937, 1932, 1936, 1934, 1938, 1931, 1931, 1935, 1939, 1934, 1935, 1934, 1940, 1947, 1935, 1943, 1941, 1943, 1945, 1941, 1952, 1946, 1950, 1427, 1433, 1443, 1443, 1444, 1423, 1390, 1368, 1170, 1178, 1171, 1177, 1187, 1161, 1130, 1097, 1536, 1547, 1563, 1596, 1619, 1638, 1663, 2001, 1999, 1995, 2003, 2003, 2001, 2003, 1997, 1994, 1996, 1990, 1998, 1991, 1998, 2001, 1986, 1990, 1988, 1987, 1986, 1982, 1988, 1988, 1982, 1955, 1957, 1961, 1959, 1959, 1952, 1957, 1959, 1944, 1957, 1953, 1952, 1952, 1947, 1955, 1939, 1947, 1942, 1955, 1946, 1947, 1943, 1940, 1935, 1945, 1937, 1941, 1939, 1949, 1938, 1951, 1997, 2042, 2115, 2212, 2279, 2356, 2407, 2895, 2918, 2926, 2914, 2885, 2868, 2825, 2527, 2509, 2490, 2455, 2421, 2406, 2379, 2355, 2041, 2034, 2007, 1994, 1977, 1952, 1946, 1891, 1884, 1891, 1888, 1890, 1892, 1887, 1891, 1891, 1887, 1889, 1887, 1886, 1890, 1891, 1914, 1911, 1911, 1919, 1921, 1914, 1919, 1926, 1927, 1923, 1927, 1919, 1922, 1922, 1923, 1931, 1937, 1933, 1936, 1947, 1925, 1940, 1939, 1941, 1936, 1933, 1942, 1942, 1927, 1966, 1970, 1972, 1963, 1970, 1970, 1976, 1977, 1314, 1290, 1286, 1254, 1261, 1264, 1263, 1283, 1260, 1272, 1265, 1259, 1263, 1278, 1687, 1691, 1693, 1691, 1688, 1700, 1710, 1998, 2010, 2005, 1994, 2003, 1999, 2002, 1998, 1993, 1991, 1991, 1995, 1989, 1994, 1990, 1984, 1983, 1985, 1992, 1982, 1982, 1975, 1956, 1951, 1957, 1950, 1949, 1959, 1955, 1958, 1949, 1945, 1953, 1947, 1951, 1949, 1945, 1938, 1947, 1932, 1948, 1941, 1941, 1938, 1939, 1923, 1939, 1947, 1937, 1945, 2095, 2148, 2191, 2231, 2248, 2283, 2296, 2759, 2767, 2769, 2779, 2798, 2843, 2877, 2486, 2489, 2470, 2459, 2473, 2436, 2423, 2013, 1988, 2002, 1995, 2011, 1999, 1891, 1889, 1889, 1883, 1885, 1891, 1883, 1895, 1875, 1894, 1890, 1889, 1897, 1888, 1912, 1910, 1920, 1913, 1914, 1916, 1916, 1919, 1917, 1918, 1924, 1925, 1926, 1925, 1933, 1939, 1933, 1935, 1935, 1931, 1930, 1940, 1940, 1943, 1936, 1940, 1945, 1945, 1945, 1958, 1965, 1959, 1961, 1973, 1419, 1391, 1336, 1274, 1255, 1246, 1251, 1071, 1065, 1073, 1112, 1155, 1197, 1551, 1585, 1609, 1628, 1649, 1656, 1691, 1966
    };

        private void Form1_Load(object sender, EventArgs e)
        {

        }
    }


  

}


