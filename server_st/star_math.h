#ifndef STAR_MATH_H
#define STAR_MATH_H
#include <cmath>
#include <QDateTime>
#include <QDebug>
#include <limits>
#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define ms_per_day  (1000 * 24 * 60 * 60)

template<class T>
inline T degrees(T rad)
{
    return rad * static_cast<T>(RAD_TO_DEG);
};

template<class T>
inline T radians(T deg)
{
    return deg * static_cast<T>(DEG_TO_RAD);
};

//references: http://www.stargazing.net/kepler/altaz.html
//http://star-www.st-and.ac.uk/~fv/webnotes/chapter7.htm
//https://en.wikipedia.org/wiki/Sidereal_time
//sideral second != solar second(regular)!!!!

double inline calcUT(const QDateTime& now)
{
    double hr_fraq = now.time().minute() / 60.;
    double UT_dayfrag  = (now.time().hour() + hr_fraq) / 24.;
    double UT  = now.date().toJulianDay() -  2451545.5; //not sure where from .5 comes >:
    return UT + UT_dayfrag;
}

template <typename T>
typename std::enable_if<std::is_unsigned<T>::value, int>::type
inline constexpr signum(T x) {
    return T(0) < x;
}

template <typename T>
typename std::enable_if<std::is_signed<T>::value, int>::type
inline constexpr signum(T x)
{
    return (T(0) < x) - (x < T(0));
}

double inline cutOffCircle(double value, int circle_lentght) //360 or 24
{
    long rounds = std::abs(value) / std::abs(circle_lentght);
    return (std::abs(value) - rounds * std::abs(circle_lentght)) * signum(value);
}

double inline getLSTDegrees(double longitudeDegrees)
{
    auto now        = QDateTime::currentDateTimeUtc();
    double GMST_hrs = cutOffCircle(18.697374558 + 24.06570982441908 * calcUT(now), 24);
    double lst      = cutOffCircle(15. * GMST_hrs + longitudeDegrees, 360);
    if(lst < 0) lst += 360;
    return lst;
}

void inline convertRA_AZ(const double ra_rad, const double dec_rad, const double latitude_rad, const double longitude_rad, double& azimuth_rad, double& alt_rad)
{
    auto lst_rad = radians(getLSTDegrees(degrees(longitude_rad)));
    auto HA = lst_rad - ra_rad;
    if (HA < 0)
        HA += 2 * M_PI;
    const auto sin_alt = sin(dec_rad) * sin(latitude_rad) + cos(dec_rad) * cos(latitude_rad) * cos (HA);
    alt_rad = asin(sin_alt);

    const auto a = acos((sin(dec_rad) - sin_alt * sin(latitude_rad)) / (cos(alt_rad) * cos(latitude_rad)));
    azimuth_rad = (sin(HA) < 0) ? a : (2 * M_PI - a);
}

//http://star-www.st-and.ac.uk/~fv/webnotes/chapter7.htm
void inline convertAZ_RA(const double azimuth_rad, const double alt_rad, const double latitude_rad, const double longitude_rad, double& ra_rad, double& dec_rad)
{
    double sin_th = sin(alt_rad) * sin(latitude_rad) + cos(alt_rad) * cos(latitude_rad) * cos (azimuth_rad);
    dec_rad = asin(sin_th);

    //double sin_H  = -sin(azimuth_rad) * cos(alt_rad) / cos(dec_rad);
    double cos_H = (sin(alt_rad) - sin(latitude_rad) * sin_th) / (cos(dec_rad) * cos(latitude_rad));
    double H = acos(cos_H);
    //double H = asin(sin_H);

    //qDebug() << "H : "<<H;
    ra_rad = radians(getLSTDegrees(degrees(longitude_rad))) - H;
}

#endif // STAR_MATH_H
