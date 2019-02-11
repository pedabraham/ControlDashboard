import control
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, step2
from flask import Flask, render_template, request, Response
import io


def poloDominante(Mp1, ta):
    Mp = Mp1 / 100
    L = np.log(Mp)**2
    E = np.sqrt(L / (np.pi**2 + L))
    Wn = 4 / (E * ta)
    polosDominantes = np.roots([1, 2 * E * Wn, Wn**2])
    print(ta)
    print ('poloDominante: '+str(polosDominantes[0]))
    return polosDominantes[0]


def mod(n):
    return np.sqrt(np.real(n)**2 + np.imag(n)**2)


def controlador(G, accion, PoloD):
    PD = control.TransferFunction([1], [1])
    PI = control.TransferFunction([1], [1, 0])
    PID = control.TransferFunction([1, 9.37], [1, 0])
    if accion == "PD":
        G = G * PD
    elif accion == "PI" or accion == "I":
        G = G * PI
    elif accion == "PID":
        G = G * PID

    angulosPolo = 0
    for polo in G.pole():
        if np.real(polo) > np.real(PoloD):
            angulosPolo += np.pi - \
                np.arctan(np.imag(PoloD) /
                          np.abs(np.real(polo) - np.real(PoloD)))
        else:
            angulosPolo += np.arctan(np.imag(PoloD) /
                                     np.abs(np.real(polo) - np.real(PoloD)))

        #print (np.degrees(angulosPolo))

    angulosZero = 0
    for zero in G.zero():
        if np.real(zero) > np.real(PoloD):
            angulosZero += np.pi - \
                np.arctan(np.imag(PoloD) /
                          np.abs(np.real(polo) - np.real(PoloD)))
        else:
            angulosZero += np.arctan(np.imag(PoloD) /
                                     np.abs(np.real(polo) - np.real(PoloD)))

    if (accion == "P" or accion == "I") and (angulosPolo - angulosZero) != np.pi:
        print("La accion de control " + accion +
              " no funciona porque no cumple la condicion de fase")
        return 0

    anguloZeroPD = -np.pi + angulosPolo - angulosZero
    # print(np.degrees(anguloZeroPD))
    distanceNewZero = np.real(PoloD) - np.imag(PoloD) / np.tan(anguloZeroPD)

    GcwK = control.TransferFunction([1, -distanceNewZero], [1])

    mDen = 1
    mNum = 1
    for polo in (GcwK * G).pole():
        mDen = mDen * mod(polo - PoloD)

    for zero in (GcwK * G).zero():
        mNum = mNum * mod(zero - PoloD)
    k = mDen / mNum

    GcwK = GcwK * k
    T, yout = control.step_response(GcwK * G / (GcwK * G + 1))
    # plt.plot(T,yout)

    # plt.show()

    if accion == "PD":
        Td = 1 / (-distanceNewZero)
        Kp = k / Td
        sal = "k= " + str(k) + " kp= " + str(Kp) + " Td= " + str(Td)
        print(sal)
    elif accion == "PID":
        Td = 1 / (-distanceNewZero - PID.zero())
        Kp = k / Td
        Ti = 1 / (distanceNewZero * PID.zero() * Td)
        sal = ("k= " + str(k) + " kp= " + str(Kp) +
               " Td= " + str(Td) + " Ti= " + str(Ti))
        print(sal)
    elif accion == "PI":
        Ti = 1 / (-distanceNewZero)
        Kp = k / Ti
        sal = ("k= " + str(k) + " kp= " + str(Kp) + " Ti= " + str(Ti))
        print(sal)
    else:
        print("No se tiene definicion de la accion de control deseada; las acciones de control definidas son I, P, PI, PD y PID")
    return(T, yout, sal)


app = Flask(__name__)


@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        # Then get the data from the form

        GPlantaStrInput = request.form['G']
        GPlantaStr = GPlantaStrInput.split("/")

        num = [float(i) for i in GPlantaStr[0].split(',')]
        if len(GPlantaStr) >= 2:
            den = [float(i) for i in GPlantaStr[1].split(',')]
        else:
            den = [1.0]
        GPlanta = control.TransferFunction(num, den)
        accion = request.form['action']
        Ta = request.form['ta']
        Mp = request.form['mp']
        #respt = controlador(GPlanta, accion, -2 + 2.5j)
        if float(Mp)>0 and float(Ta)>0:
            respt = controlador(GPlanta, accion, poloDominante(float(Mp),float(Ta)))
        else:
            respt = controlador(GPlanta, accion, -2 + 2.5j) # NOTE: No hay tiempos en 0 o menores por lo que solo se ponen unos polos dominantes de referencia

        #respuesta = "request.form['vel']"
        #respuesta2 = request.form['u']
        # print(GPlanta)

        return render_template('index.html', ta=Ta, mp=Mp, a=respt[0].tolist(), b=respt[1].tolist(), planta=GPlantaStrInput, constantes=respt[2])
    else:
        return render_template('index.html', ta='0', mp='9', a=[1, 2, 3], b=[1, 0, 3], planta='1/1,9.21,19.89,0', constantes="")

        # print(len(velocidad))
        # if len(GP) > 0:

        # print(respuesta)
        #velocidad = respuesta
        #x = int(velocidad)

        #salidaHtml = str(velocidad)


if __name__ == '__main__':
    app.run(debug=True)
