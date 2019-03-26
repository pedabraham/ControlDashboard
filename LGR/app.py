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


def controladorByLGR(G, accion, PoloD):
    Tin, yin = control.step_response(G / (G + 1))
    rlistI, klistI = control.root_locus(G,Plot=False)
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
    Tout, yout = control.step_response(GcwK * G / (GcwK * G + 1))
    rlistO, klistO = control.root_locus(GcwK * G,Plot=False)
    # plt.plot(T,yout)

    # plt.show()


    if accion == "PD":
        Td = 1 / (-distanceNewZero)
        Kp = k / Td
        sal = "k= " + str(k) + " kp= " + str(Kp) + " Td= " + str(Td)
        constantes = {
            'ti' : 0,
            'td' : Td,
            'kp' : Kp
        }
        print(sal)
    elif accion == "PID":
        Td = (1 / (-distanceNewZero - PID.zero()))[0]
        Kp = k / Td
        Ti = (1 / (distanceNewZero * PID.zero() * Td))[0]
        sal = ("k= " + str(k) + " kp= " + str(Kp) +
               " Td= " + str(Td) + " Ti= " + str(Ti))
        print(sal)
        constantes = {
            'ti' : Ti,
            'td' : Td,
            'kp' : Kp
        }

    elif accion == "PI":
        Ti = 1 / (-distanceNewZero)
        Kp = k / Ti
        sal = ("k= " + str(k) + " kp= " + str(Kp) + " Ti= " + str(Ti))
        print(sal)
        constantes = {
            'ti' : Ti,
            'td' : 0,
            'kp' : Kp
        }

    else:
        print("No se tiene definicion de la accion de control deseada; las acciones de control definidas son I, P, PI, PD y PID")
        constantes = {
            'ti' : 0,
            'td' : 0,
            'kp' : 0
        }
    return(Tout, yout, sal,Tin,yin,rlistI,rlistO,constantes)

def controladorByFreq(planta,accion,tr,fase):
  planta = planta.minreal()
  Tin, yin = control.step_response(planta/(planta+1))
  magIn, phaseIn, omegaIn = control.bode(planta,Plot=False)
  PD = control.TransferFunction([1],[1])
  PI = control.TransferFunction([1],[1,0])
  PID = control.TransferFunction([1,5],[1,0])
  if accion == "PD":
        G = planta * PD
  elif accion == "PI" or accion == "I":
        G = planta * PI
  elif accion == "PID":
        G = planta * PID
  G = G.minreal()
  wc = 1/tr
  faseR = (fase/180)*np.pi
  tfgf = control.TransferFunction([1],[1]) #transfer function gain finder
  for zero in G.zero():
      tfp = control.TransferFunction([1],[1,-1*zero])#transfer funtion of prube
      tfgf = tfgf * tfp
  tfg = G * tfgf
  gain = tfg.num[0][0][0]
  angP = (sum(np.arctan(wc/(-1*np.array(list(filter(lambda x: x < 0, G.pole()))))))+
    0.5*np.pi*len(list(filter(lambda x: x == 0, G.pole()))))
  angZ = (sum(np.arctan(wc/(-1*np.array(list(filter(lambda x: x < 0, G.zero()))))))+
      0.5*np.pi*len(list(filter(lambda x: x == 0, G.zero()))))
  wpd = wc/(np.tan(faseR-np.pi+angP-angZ))
  td = 1 /wpd
  kp = (((np.prod(np.sqrt((wc/(-1*np.array(list(filter(lambda x: x < 0, G.pole())))))**2+1)))/
      (np.prod(np.sqrt((wc/(-1*np.array(list(filter(lambda x: x < 0, np.append(G.zero(),wpd)))))**2+1)))))*
        (np.prod(-1*np.array(list(filter(lambda x: x < 0, G.pole()))))/
         (np.prod(-1*np.array(list(filter(lambda x: x < 0, G.zero()))))*gain)))
  ti = td
  TdPID = 1 / ((wc/wpd) +(-1* PID.zero()))
  TiPID = 1 / ((wc/wpd) * (-1*PID.zero()) * TdPID)
  KP_pid = kp/TdPID
  Gc = kp * control.TransferFunction([wc/wpd,1],[1])
  Tout, yout = control.step_response(G*Gc/(G*Gc+1))
  magOut, phaseOut, omegaOut = control.bode(G*Gc,Plot=False)
  if accion == "PD":
        sal = " kp= " + str(kp) + " Td= " + str(td)
        print(sal)
        constantes = {
            'ti' : 0,
            'td' : td,
            'kp' : kp
        }
  elif accion == "PID":
        sal = (" kp= " + str(KP_pid[0]) +
               " Td= " + str(TdPID[0]) + " Ti= " + str(TiPID[0]))
        print(sal)
        constantes = {
            'ti' : TiPID[0],
            'td' : TdPID[0],
            'kp' : KP_pid[0]
        }
  elif accion == "PI":
        sal = (" kp= " + str(kp) + " Ti= " + str(ti))
        print(sal)
        constantes = {
            'ti' : ti,
            'td' : 0,
            'kp' : kp
        }
  else:
        print("No se tiene definicion de la accion de control deseada; las acciones de control definidas son I, P, PI, PD y PID")
  return (sal,Tin, yin,Tout, yout,constantes)

def setValoresEqDif(constantes,accion,T):
    if accion == "PD":
        Td = constantes['td']
        Kp = constantes['kp']
        print(Td)
        print(Kp)


        a0 = (Kp + Td/T)
        a1 = -Td/T
        a2 = 0
        b0 = 0
        print('ved')
        print(a0)

        return( {
            'a0': a0,
            'a1': a1,
            'a2': a2,
            'b0': 0
        })

    elif accion == "PID":
        Td = constantes['td']
        Kp = constantes['kp']
        Ti = constantes['ti']
        return ({
            'a0': Kp+Ti*T+Td/T,
            'a1': -Kp+2*Td/T,
            'a2': Td/T,
            'b0': 1
        })
    elif accion == "PI":
        Kp = constantes['kp']
        Ti = constantes['ti']
        return  ({
            'a0': Kp,
            'a1': Ti*T-Kp,
            'a2': 0,
            'b0': 1
        })
    else:
        return ({
            'a0': 0,
            'a1': 0,
            'a2': 0,
            'b0': 0
        })
    print(valoresEqDif)
    return valoresEqDif

app = Flask(__name__)

GPlantaStrInput = '1/1,9.21,19.89,0'
Mp = '9'
Ta='0.1'
Ts = '0.05'
tr = '8'
fase = '60'
accion = 'PD'
T = 0.05

valoresEqDif = {
    'a0': 0,
    'a1': 0,
    'a2': 0,
    'b0': 0,
}

constantes = {
    'ti' : 0,
    'td' : 0,
    'kp' : 0
}

@app.route('/', methods=['GET', 'POST'])
def index():
    webC = {'estado':'none'}
    if request.method == 'POST':
        # Then get the data from the form

        global GPlantaStrInput,Mp,Ta,T,constantes,accion,Ts,valoresEqDif
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
        Ts = request.form['ts']

        #respt = controladorByLGR(GPlanta, accion, -2 + 2.5j)
        if len(Ta)>0 and len(Mp)>0 and len(Ts)>0:
            T = float(Ts)
            if float(Mp)>0 and float(Ta)>0 :
                to,yo,sal,ti,yi,rI,rO,constantes = controladorByLGR(GPlanta, accion, poloDominante(float(Mp),float(Ta)))
            else:
                to,yo,sal,ti,yi,rI,rO,constantes = controladorByLGR(GPlanta, accion, -2 + 2.5j) # NOTE: No hay tiempos en 0 o menores por lo que solo se ponen unos polos dominantes de referencia

        rI  =   list(zip(*rI))
        realI = np.real(rI)
        imagI = np.imag(rI)
        rO  =   list(zip(*rO))
        realO = np.real(rO)
        imagO = np.imag(rO)
        webC['estado']='inline'
        valoresEqDif = setValoresEqDif(constantes,accion,T)
        print(str(valoresEqDif))
        #respuesta = "request.form['vel']"
        #respuesta2 = request.form['u']
        # print(GPlanta)

        return render_template('index.html',estado=webC, valoresEqDif=str(valoresEqDif), ta=Ta, mp=Mp, ts=Ts, to=[to.tolist()], yo=[yo.tolist()], planta=GPlantaStrInput, constantes=sal,
         ti = [ti.tolist()],yi=[yi.tolist()],realI=realI.tolist(),imagI=imagI.tolist(),realO=realO.tolist(),imagO=imagO.tolist())
    else:
        return render_template('index.html',estado=webC,valoresEqDif='', ta=Ta, mp=Mp, ts=Ts, to=[[1, 2, 3]], yo=[[1, 0, 3]], planta=GPlantaStrInput, constantes="",
        ti = [[1,1.5,2]],yi=[[3,6,7]], realI=[[2,3,1],[4,2,3],[9,6,2]],imagI=[[4,5,3],[4,8,9],[3,1,0]],realO = [[1,3]],imagO=[[4,5]])

        # print(len(velocidad))
        # if len(GP) > 0:

        # print(respuesta)
        #velocidad = respuesta
        #x = int(velocidad)

        #salidaHtml = str(velocidad)
@app.route('/freq', methods=['GET','POST'])
def freq():
    webC = {'estado':'none'}
    if request.method == 'POST':
        # Then get the data from the form
        global GPlantaStrInput,tr,fase,T,Ts
        GPlantaStrInput = request.form['G']
        GPlantaStr = GPlantaStrInput.split("/")
        num = [float(i) for i in GPlantaStr[0].split(',')]
        if len(GPlantaStr) >= 2:
            den = [float(i) for i in GPlantaStr[1].split(',')]
        else:
            den = [1.0]
        GPlanta = control.TransferFunction(num, den)
        accion = request.form['action']
        tr = request.form['tr']
        fase = request.form['fase']
        Ts = request.form['ts']
        T = float(Ts)
        #respt = controladorByLGR(GPlanta, accion, -2 + 2.5j)
        if float(tr)>0 and float(fase)>0:
            sal,Tin, yin,Tout, yout,constantes = controladorByFreq(GPlanta, accion, float(tr),float(fase))
        else:
            sal,Tin, yin,Tout, yout,constantes = controladorByFreq(GPlanta, accion, 5,6) # NOTE: No hay tiempos en 0 o menores por lo que solo se ponen unos polos dominantes de referencia

        """rI  =   list(zip(*rI))
        realI = np.real(rI)
        imagI = np.imag(rI)
        rO  =   list(zip(*rO))
        realO = np.real(rO)
        imagO = np.imag(rO)"""

        valoresEqDif = setValoresEqDif(constantes,accion,T)
        webC['estado']='inline'
        #respuesta = "request.form['vel']"
        #respuesta2 = request.form['u']
        # print(GPlanta)

        return render_template('freq.html',ts=Ts, estado=webC, valoresEqDif=str(valoresEqDif), planta=GPlantaStrInput, tr=tr, fase=fase,constantes=sal,Tin=[Tin.tolist()], yin=[yin.tolist()],Tout=[Tout.tolist()], yout=[yout.tolist()])
    else:
        return render_template('freq.html',ts=Ts, estado=webC, valoresEqDif='', planta=GPlantaStrInput, tr=tr, fase=fase,constantes="",Tin=[[5,2]], yin=[[6,3]],Tout=[[7,4,1],[6,9]], yout=[[5,2,9],[7.89,5]])

@app.route('/controlar', methods=['GET','POST'])
def usoDeControlador():
    global constantes,accion

    valoresEqDif= setValoresEqDif(constantes,accion,T)
    print(constantes)
    print(accion)
    print(valoresEqDif)
    return render_template('usoDeControlador.html', valoresEqDif=valoresEqDif)

if __name__ == '__main__':
    app.run(debug=True)
