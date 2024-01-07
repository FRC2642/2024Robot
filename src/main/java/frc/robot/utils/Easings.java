
package frc.robot.utils;


public class Easings {

    public enum Functions {
        easeLinear,
        easeStep,
        easeInQuad,
        easeOutQuad,
        easeInOutQuad,
        easeInCubic,
        easeOutCubic,
        easeInOutCubic,
        easeInQuart,
        easeOutQuart,
        easeInOutQuart,
        easeInQuint,
        easeOutQuint,
        easeInOutQuint,
        easeInSine,
        easeOutSine,
        easeInOutSine,
        easeInCirc,
        easeOutCirc,
        easeInOutCirc,
        easeInExpo,
        easeOutExpo,
        easeInOutExpo,
        easeInElastic,
        easeOutElastic,
        easeInOutElastic,
        easeInBack,
        easeOutBack,
        easeInOutBack,
        easeInBounce,
        easeOutBounce,
        easeInOutBounce
    }

    private static final double PI = Math.PI;
    private static final double HALFPI = Math.PI / 2.0f;

    public static double interpolate(double outputMin, double outputMax, double inputMin, double inputMax, double input, Functions easing) {
        double normalizedInput = MathR.lerp(0, 1, inputMin, inputMax, input);
        double interpolated = interpolate(normalizedInput, easing);
        double denormalized = MathR.lerp(outputMin, outputMax, 0, 1, interpolated);
        return denormalized;
    }

    /// Interpolate using the specified function.
    public static double interpolate(double p, Functions function) {
        switch (function) {
            case easeStep:
                return easeStep(p);
            case easeOutQuad:
                return easeOutQuad(p);
            case easeInQuad:
                return easeInQuad(p);
            case easeInOutQuad:
                return easeInOutQuad(p);
            case easeInCubic:
                return easeInCubic(p);
            case easeOutCubic:
                return easeOutCubic(p);
            case easeInOutCubic:
                return easeInOutCubic(p);
            case easeInQuart:
                return easeInQuart(p);
            case easeOutQuart:
                return easeOutQuart(p);
            case easeInOutQuart:
                return easeInOutQuart(p);
            case easeInQuint:
                return easeInQuint(p);
            case easeOutQuint:
                return easeOutQuint(p);
            case easeInOutQuint:
                return easeInOutQuint(p);
            case easeInSine:
                return easeInSine(p);
            case easeOutSine:
                return easeOutSine(p);
            case easeInOutSine:
                return easeInOutSine(p);
            case easeInCirc:
                return easeInCirc(p);
            case easeOutCirc:
                return easeOutCirc(p);
            case easeInOutCirc:
                return easeInOutCirc(p);
            case easeInExpo:
                return easeInExpo(p);
            case easeOutExpo:
                return easeOutExpo(p);
            case easeInOutExpo:
                return easeInOutExpo(p);
            case easeInElastic:
                return easeInElastic(p);
            case easeOutElastic:
                return easeOutElastic(p);
            case easeInOutElastic:
                return easeInOutElastic(p);
            case easeInBack:
                return easeInBack(p);
            case easeOutBack:
                return easeOutBack(p);
            case easeInOutBack:
                return easeInOutBack(p);
            case easeInBounce:
                return easeInBounce(p);
            case easeOutBounce:
                return easeOutBounce(p);
            case easeInOutBounce:
                return easeInOutBounce(p);
            default:
                return easeLinear(p);
        }
    }

    public static double easeLinear(double p) {
        return p;
    }

    // It's either 1, or it's not
    public static double easeStep(double p) {
        return Math.floor(p);
    }

    public static double easeInQuad(double p) {
        return p * p;
    }

    /// Modeled after the parabola y = -x^2 + 2x
    public static double easeOutQuad(double p) {
        return -(p * (p - 2));
    }

    // Modeled after the piecewise quad
    // y = (1/2)((2x)^2) ; [0, 0.5)
    // y = -(1/2)((2x-1)*(2x-3) - 1) ; [0.5, 1]
    public static double easeInOutQuad(double p) {
        if (p < 0.5f) {
            return 2 * p * p;
        }

        return (-2 * p * p) + (4 * p) - 1;
    }

    // Modeled after the cubic y = x^3
    public static double easeInCubic(double p) {
        return p * p * p;
    }

    // Modeled after the cubic y = (x - 1)^3 + 1
   
    public static double easeOutCubic(double p) {
        double f = p - 1;
        return (f * f * f) + 1;
    }

    
    // Modeled after the piecewise cubic
    // y = (1/2)((2x)^3) ; [0, 0.5)
    // y = (1/2)((2x-2)^3 + 2) ; [0.5, 1]
    public static double easeInOutCubic(double p) {
        if (p < 0.5f) {
            return 4 * p * p * p;
        }

        double f = (2 * p) - 2;
        return (0.5f * f * f * f) + 1;
    }

    // Modeled after the quart x^4
    public static double easeInQuart(double p) {
        return p * p * p * p;
    }

    // Modeled after the quart y = 1 - (x - 1)^4
    public static double easeOutQuart(double p) {
        double f = p - 1;
        return (f * f * f * (1 - p)) + 1;
    }

    // Modeled after the piecewise quart
    // y = (1/2)((2x)^4) ; [0, 0.5)
    // y = -(1/2)((2x-2)^4 - 2) ; [0.5, 1]
    public static double easeInOutQuart(double p) {
        if (p < 0.5f) {
            return 8 * p * p * p * p;
        }

        double f = p - 1;
        return (-8 * f * f * f * f) + 1;
    }

    // Modeled after the quint y = x^5
    // </summary>
    public static double easeInQuint(double p) {
        return p * p * p * p * p;
    }

    // Modeled after the quint y = (x - 1)^5 + 1
    public static double easeOutQuint(double p) {
        double f = p - 1;
        return (f * f * f * f * f) + 1;
    }

    // Modeled after the piecewise quint
    // y = (1/2)((2x)^5) ; [0, 0.5)
    // y = (1/2)((2x-2)^5 + 2) ; [0.5, 1]
    public static double easeInOutQuint(double p) {
        if (p < 0.5f) {
            return 16 * p * p * p * p * p;
        }

        double f = (2 * p) - 2;
        return (0.5f * f * f * f * f * f) + 1;
    }

    // Modeled after quarter-cycle of sine wave
    public static double easeInSine(double p) {
        return Math.sin((p - 1) * HALFPI) + 1;
    }

    // Modeled after quarter-cycle of sine wave (different phase)
    public static double easeOutSine(double p) {
        return Math.sin(p * HALFPI);
    }

    // Modeled after half sine wave
    public static double easeInOutSine(double p) {
        return 0.5f * (1 - Math.cos(p * PI));
    }

    // Modeled after shifted quadrant IV of unit circle
    public static double easeInCirc(double p) {
        return 1 - Math.sqrt(1 - (p * p));
    }

    // Modeled after shifted quadrant II of unit circle
    public static double easeOutCirc(double p) {
        return Math.sqrt((2 - p) * p);
    }

    // Modeled after the piecewise circ function
    // y = (1/2)(1 - Math.sqrt(1 - 4x^2)) ; [0, 0.5)
    // y = (1/2)(Math.sqrt(-(2x - 3)*(2x - 1)) + 1) ; [0.5, 1]
    public static double easeInOutCirc(double p) {
        if (p < 0.5f) {
            return 0.5f * (1 - Math.sqrt(1 - (4 * (p * p))));
        }

        return 0.5f * (Math.sqrt(-((2 * p) - 3) * ((2 * p) - 1)) + 1);
    }

    // Modeled after the expo function y = 2^(10(x - 1))
    public static double easeInExpo(double p) {
        return (p <= 0.0f) ? p : Math.pow(2, 10 * (p - 1));
    }

    // Modeled after the expo function y = -2^(-10x) + 1
    public static double easeOutExpo(double p) {
        return (p >= 1.0f) ? p : 1 - Math.pow(2, -10 * p);
    }

    // Modeled after the piecewise expo
    // y = (1/2)2^(10(2x - 1)) ; [0,0.5)
    // y = -(1/2)*2^(-10(2x - 1))) + 1 ; [0.5,1]
    public static double easeInOutExpo(double p) {
        if (p == 0.0 || p >= 1.0) {
            return p;
        }

        if (p < 0.5f) {
            return 0.5f * Math.pow(2, (20 * p) - 10);
        }

        return (-0.5f * Math.pow(2, (-20 * p) + 10)) + 1;
    }

    // Modeled after the damped sine wave y = sin(13pi/2*x)*Math.pow(2, 10 * (x -
    // 1))
    public static double easeInElastic(double p) {
        return Math.sin(13 * HALFPI * p) * Math.pow(2, 10 * (p - 1));
    }

    // Modeled after the damped sine wave y = sin(-13pi/2*(x + 1))*Math.pow(2,
    // -10x) + 1
    public static double easeOutElastic(double p) {
        return (Math.sin(-13 * HALFPI * (p + 1)) * Math.pow(2, -10 * p)) + 1;
    }

    // Modeled after the piecewise expoly-damped sine wave:
    // y = (1/2)*sin(13pi/2*(2*x))*Math.pow(2, 10 * ((2*x) - 1)) ; [0,0.5)
    // y = (1/2)*(sin(-13pi/2*((2x-1)+1))*Math.pow(2,-10(2*x-1)) + 2) ; [0.5, 1]
    public static double easeInOutElastic(double p) {
        if (p < 0.5f) {
            return 0.5f * Math.sin(13 * HALFPI * (2 * p)) * Math.pow(2, 10 * ((2 * p) - 1));
        }

        return 0.5f * ((Math.sin(-13 * HALFPI * (2 * p)) * Math.pow(2, -10 * ((2 * p) - 1))) + 2);
    }

    // Modeled after the overshooting cubic y = x^3-x*sin(x*pi)
    public static double easeInBack(double p) {
        return (p * p * p) - (p * Math.sin(p * PI));
    }

    // Modeled after overshooting cubic y = 1-((1-x)^3-(1-x)*sin((1-x)*pi))
    public static double easeOutBack(double p) {
        double f = 1 - p;
        return 1 - ((f * f * f) - (f * Math.sin(f * PI)));
    }

    // Modeled after the piecewise overshooting cubic function:
    // y = (1/2)*((2x)^3-(2x)*sin(2*x*pi)) ; [0, 0.5)
    // y = (1/2)*(1-((1-x)^3-(1-x)*sin((1-x)*pi))+1) ; [0.5, 1]
    public static double easeInOutBack(double p) {
        if (p < 0.5f) {
            double f = 2 * p;
            return 0.5f * ((f * f * f) - (f * Math.sin(f * PI)));
        } else {
            double f = 1 - ((2 * p) - 1);
            return (0.5f * (1 - ((f * f * f) - (f * Math.sin(f * PI))))) + 0.5f;
        }
    }

    public static double easeInBounce(double p) {
        return 1 - easeOutBounce(1 - p);
    }

    public static double easeOutBounce(double p) {
        if (p < 4 / 11.0f)
            return 121 * p * p / 16.0f;
        if (p < 8 / 11.0f)
            return (363 / 40.0f * p * p) - (99 / 10.0f * p) + (17 / 5.0f);
        if (p < 9 / 10.0f)
            return (4356 / 361.0f * p * p) - (35442 / 1805.0f * p) + (16061 / 1805.0f);
        return (54 / 5.0f * p * p) - (513 / 25.0f * p) + (268 / 25.0f);
    }

    public static double easeInOutBounce(double p) {
        if (p < 0.5f) {
            return 0.5f * easeInBounce(p * 2);
        }

        return (0.5f * easeOutBounce((p * 2) - 1)) + 0.5f;
    }
}
