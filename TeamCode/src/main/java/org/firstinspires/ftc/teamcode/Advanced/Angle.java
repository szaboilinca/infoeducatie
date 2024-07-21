package org.firstinspires.ftc.teamcode.Advanced;

//Această clasă vă permite să evitați conversiile unghiurilor
//diferite tipuri de unghi sunt introduse în program și sunt necesare pentru calcule (ex. citirea IMU, matematica vectorială...)

//definiții:

//CW = în sensul acelor de ceasornic, CCW = în sens invers acelor de ceasornic

//zero este înainte (axa y pozitivă), CW este pozitiv, CCW este negativ
//zero este dreapta (pozitiv axa x), CCW este pozitiv, CW este negativ

//ZERO_TO_360_HEADING:ocolirea CW de la 0 te duce la 360 și înapoi la 0
//NEG_180_TO_180_HEADING: trecerea CW de la 0 te duce la +180, mergând CCW te duce la -180
//ZERO_TO_360_CARTESIAN: mergând CCW de la 0 te duce până la 360 și înapoi la 0
//NEG_180_TO_180_CARTESIAN: mergând în sens invers de la 0 te duce până la +180, mergând în dreapta te duce la -180


public class Angle {

    //față de poziția de pornire a robotului (dreapta = est, stânga = vest, înainte = nord, înapoi = sud)
    public static final Angle
            RIGHT = new Angle(90, AngleType.NEG_180_TO_180_HEADING),
            LEFT = new Angle(-90, AngleType.NEG_180_TO_180_HEADING),
            BACKWARD = new Angle(180, AngleType.NEG_180_TO_180_HEADING),
            FORWARD = new Angle(0, AngleType.NEG_180_TO_180_HEADING);


    enum AngleType {
        ZERO_TO_360_CARTESIAN, ZERO_TO_360_HEADING, NEG_180_TO_180_CARTESIAN, NEG_180_TO_180_HEADING
    }

    enum Direction {
        CLOCKWISE, COUNTER_CLOCKWISE
    }

    private double angle;
    private final AngleType type;

    public Angle (double angle, AngleType type) {
        this.angle = angle;
        this.type = type;

        this.angle = convertAngleDouble(type);
    }

    public double getAngle (AngleType type) {
        return this.convertAngle(type).getAngle();
    }

    public double getAngle () { return angle; }
    public AngleType getType () { return type; }

    @Override
    public String toString () {
        return "" + angle + " :" + type;
    }

    //nu există alte ipoteze legate de valoarea inputAngle (poate fi de la -infinit la infinit)
    public Angle convertAngle (AngleType outputType) {
        return new Angle (convertAngleDouble(outputType), outputType);
    }

    public double convertAngleDouble (AngleType outputType) {

        if (type == outputType) {
            return wrapAngle(this.getAngle(), outputType); //was new Angle(angle, type)
        }

        if (sameNumericalSystem(type, outputType)) {
            return convertCoordinateSystem(angle, type, outputType);
        }
        else if (sameCoordinateSystem(type, outputType)) {
            return convertNumericalSystem(angle, type, outputType);
        }
        else {

            double angleNewNumericalSystem = convertNumericalSystem(angle, type, numericalAndCoordinate(outputType, type)); //was type, output type
            double angleNewCoordinateSystem = convertCoordinateSystem(angleNewNumericalSystem, numericalAndCoordinate(outputType, type), outputType); //was type, output type
            return angleNewCoordinateSystem;
        }
    }

    // returnează valoarea absolută a diferenței dintre două unghiuri (poate fi de orice tip)
    // Valoarea minimă de returnare este 0 și valoarea maximă de returnare este 180
    public double getDifference (Angle other) {
        Angle otherConverted = other.convertAngle(AngleType.ZERO_TO_360_CARTESIAN);
        Angle thisConverted = this.convertAngle(AngleType.ZERO_TO_360_CARTESIAN);

        double rawDiff = Math.abs(otherConverted.getAngle() - thisConverted.getAngle());
        if (rawDiff > 180) {
            return 360 - rawDiff; //pozitiv
        }
        return rawDiff; //intre 0 si 180
    }

    //exemplu: direcția DE LA 0 grade LA 90 de grade (ambele de tip NEG_180_TO_180_HEADING) este în sensul acelor de ceasornic
    //întoarce fie în sensul orar, fie în sens antiorar
    //este implicit în sensul acelor de ceasornic dacă unghiurile sunt identice (diferența este zero)
    public Direction directionTo (Angle other) {
        Angle otherConverted = other.convertAngle(AngleType.ZERO_TO_360_CARTESIAN);
        Angle thisConverted = this.convertAngle(AngleType.ZERO_TO_360_CARTESIAN);

        double rawDiff = Math.abs(otherConverted.getAngle() - thisConverted.getAngle());
        if (rawDiff > 180) {
            if (otherConverted.getAngle() > thisConverted.getAngle()) {
                return Direction.CLOCKWISE;
            } else {
                return Direction.COUNTER_CLOCKWISE;
            }
        } else {
            if (otherConverted.getAngle() > thisConverted.getAngle()) {
                return Direction.COUNTER_CLOCKWISE;
            } else {
                return Direction.CLOCKWISE;
            }
        }
    }

    public Angle rotateBy (double degrees, Direction direction) {
        Angle thisConverted = this.convertAngle(AngleType.ZERO_TO_360_HEADING);
        double newAngle;
        if (direction == Direction.CLOCKWISE) {
            newAngle = thisConverted.getAngle() + degrees;
        } else {
            newAngle = thisConverted.getAngle() - degrees;
        }
        return new Angle(newAngle, AngleType.ZERO_TO_360_HEADING).convertAngle(this.type);
    }

    public Angle rotateBy (double degrees) {
        return rotateBy(degrees, this.getPositiveDirection());
    }

    public static Angle getAverageAngle (Angle angle1, Angle angle2) {
        double difference = angle1.getDifference(angle2);
        Direction direction = angle1.directionTo(angle2);
        return angle1.rotateBy(difference/2.0, direction);
    }


    public static double convertCoordinateSystem (double inputAngle, AngleType inputType, AngleType outputType) {

        if (sameCoordinateSystem(inputType, outputType)) {
            return inputAngle;
        }

        if (isCartesian(inputType)) {

            if (isZeroTo360(inputType)) {
                return 360 - wrapAngle(inputAngle - 90, outputType);
            }
            else {
                return -1 * wrapAngle(inputAngle - 90, outputType);
            }
        } else { //input type is heading system
            if (isZeroTo360(inputType)) {
                return 360 - wrapAngle(inputAngle - 90, outputType);
            }
            else {
                return -1 * wrapAngle(inputAngle - 90, outputType);
            }
        }
    }


    public static double convertNumericalSystem (double inputAngle, AngleType inputType, AngleType outputType) {
        if (sameNumericalSystem(inputType, outputType)) {
            return inputAngle;
        }
        return wrapAngle(inputAngle, outputType);
    }

    public static boolean sameCoordinateSystem(AngleType firstType, AngleType secondType) {
        return isCartesian(firstType) == isCartesian(secondType);
    }

    public static boolean sameNumericalSystem(AngleType firstType, AngleType secondType) {
        return isZeroTo360(firstType) == isZeroTo360(secondType);
    }

    public static boolean isCartesian (AngleType angleType) {
        if (angleType == AngleType.ZERO_TO_360_CARTESIAN || angleType == AngleType.NEG_180_TO_180_CARTESIAN) {
            return true;
        }
        return false;
    }

    public static boolean isZeroTo360 (AngleType angleType) {
        if (angleType == AngleType.ZERO_TO_360_CARTESIAN || angleType == AngleType.ZERO_TO_360_HEADING) {
            return true;
        }
        return false;
    }

    public static AngleType numericalAndCoordinate (AngleType numericalType, AngleType coordinateType) {
        if (isZeroTo360(numericalType) && isCartesian(coordinateType)) return AngleType.ZERO_TO_360_CARTESIAN;
        else if (!isZeroTo360(numericalType) && isCartesian(coordinateType)) return AngleType.NEG_180_TO_180_CARTESIAN;
        else if (isZeroTo360(numericalType) && !isCartesian(coordinateType)) return AngleType.ZERO_TO_360_HEADING;
        else return AngleType.NEG_180_TO_180_HEADING; //!isZeroTo360(numericalType) && !isCartesian(coordinateType)
    }

    public Direction getPositiveDirection () {
        if (this.type == AngleType.NEG_180_TO_180_HEADING || this.type == AngleType.ZERO_TO_360_HEADING) {
            return Direction.CLOCKWISE;
        }
        return Direction.COUNTER_CLOCKWISE;
    }


    public static double wrapAngle(double angle, double min, double max) {
        angle = mod(angle, range(min, max));
        if (angle > max) { //won't be < min bc of second assumption
            return min + min + angle; //I have no idea why, but it seems to work for all cases under assumptions (?)
        }
        return angle;
    }


    public static double wrapAngle(double angle, AngleType outputAngleType) {
        if (isZeroTo360(outputAngleType)) {
            return wrapAngle(angle, 0, 360);
        } else {
            return wrapAngle(angle, -180, 180);
        }
    }

    public static double range (double num1, double num2) {
        return Math.abs(num1-num2);
    }

    public static double mod (double n, double m) {
        return (((n % m) + m) % m);
    }
}