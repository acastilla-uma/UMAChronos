# Viabilidad de indices de estabilidad — 2026-11-06

## Actualizacion vinculante tras recibir los PDF originales

Fuentes locales: C:/Users/alexc/Downloads/s11771-019-4179-3.pdf (14 paginas), C:/Users/alexc/Downloads/sensors-22-05656-v2.pdf (19 paginas). Texto extraido y ecuaciones inspeccionadas visualmente. La afirmacion anterior de PLTR no verificado queda SUPERADA por esta seccion.

Li pagina PDF8 / pagina impresa2361, ecuacion10:

PLTR_t0(Dt) = 2h/tw * (ay/g + phi) + 2h/(tw*g) * {[-C0*(ay-gamma*vx)-C1*gamma_dot]/(m*vx) + 2*kf*delta_dot/(m*I) + g*phi_dot} * Dt.
C0=kf+kr; C1=lf*kf-lr*kr. I es relacion de direccion, NO inercia (pagina3 indica I=20 y ecuacion11 indica delta_f=delta/I). Con control directo de angulo de rueda, delta_dot/I se sustituye por delta_f_dot sin inventar relacion de direccion.

LTR Li ecuacion8 es firmado: (Fzr-Fzl)/(Fzr+Fzl); se puede mostrar magnitud adicional, no atribuir abs a esa ecuacion8.

Disponibles/derivables: m,h,tw,velocidad longitudinal local,aceleracion lateral local,roll,yaw rate,yaw acceleration,roll rate,direccion de ruedas y sus derivadas temporales,tiempos. Horizonte Dt es eleccion de configuracion cientifica, no valor verificado por la ecuacion. El modelo usa 2/3/4 ejes y algunos multiples ejes direccionales; lf/lr y kf/kr de modelo monovia de dos ejes requieren reduccion/identificacion explicita. kf/kr son rigideces de deriva del neumatico, no coeficientes de rozamiento ni rigidez vertical del terreno. No existen como parametros del motor actual (engine.calculate_friction usa mu*N*tanh(diferencia de velocidad), server selecciona rigid mesh tire y no ejecuta TMeasy). No copiar constantes SUV del paper ni inferir kf/kr solo de mu. vx=0 hace singular la ecuacion literal; no ocultar con clamp.

Alternativa calculable con datos existentes, pendiente decision del usuario: predictor cinematico de primer orden, inspirado en Li. Sustituir termino predictivo del modelo monovia por derivada temporal de aceleracion lateral local y usar roll rate con convencion coherente. Esto NO es incondicionalmente equivalente a ecuacion10: requiere hipotesis monovia/velocidad casi constante para identificar ambos. No etiquetarlo PLTR exacto de Li. Otra alternativa es identificar/calibrar rigideces equivalentes y reducir multiples ejes, ampliacion material de alcance. El motor actual no impide toda prediccion; impide afirmar calculo literal de Li sin esas decisiones/datos.

Brecelj pagina8 ecuaciones17-20 confirmadas visualmente. Ecuacion21 y pagina9 indican que simplificar ZML a linea pasando por CoG con angulo obtenido solo de aceleracion presupone rotacion/torques despreciables. Mantener formulacion completa con fuerzas/momentos; una grafica debe identificar coordenadas/magnitud derivada en vez de inventar escalar normalizado ZML.

Entrevista continua: usuario aporto fuentes como evidencia, no selecciono politica de aproximaciones. Falta decision cientifica sobre PLTR adaptado frente a fuente literal con parametros pendientes.

Investigacion solicitada tras ronda 3. No se ha aceptado una politica de aproximaciones ni se ha implementado la aplicacion.

## Conclusiones
- LTR: calculable desde fuerzas verticales por lado de ruedas. Agrupar en marco local por identidad geometrica, no Y mundial. Excluir contactos de chasis para LTR de ruedas. Denominador nulo => no definido. Original primario para LTR (no predictor Li): https://link.springer.com/article/10.1186/s10033-019-0323-0 eq31, magnitud abs((Fzr-Fzl)/(Fzr+Fzl)); documentar convencion de signo si se ofrece.
- SI energetico: calculable desde parametros del vehiculo simulado. SI=1-k1*abs(phi)/phi_c-k2*(omega_x_body/omega_c)^2, k1=1.15,k2=2.05 heredados como coeficientes nominales sin validacion para cada vehiculo. phi_c=atan(b/h), d=hypot(b,h), omega_c=sqrt(2*m*g*(d-h)/(Ixx_cg+m*d*d)). Parametros geometricos y umbral nominales; no equivalen a criterio de seguridad validado sobre cualquier apoyo 3D. No copiar constantes del MAN completo de Chrono.
- ZML: construccion de linea factible desde wrench y geometria de contactos; no es un escalar normalizado. Articulo primario completo https://pmc.ncbi.nlm.nih.gov/articles/PMC9371053/ eq8-20. Para planos z constantes, x(z)=(-My(0)+z*Fx)/Fz; y(z)=(Mx(0)+z*Fy)/Fz. Todas las fuerzas y momentos con origen y marco consistente. El resultado requiere interpretacion con geometria de soporte. Fz=0 no definido en esta representacion. La simplificacion de angulo por aceleraciones de sec2.1.3 asume rotaciones/torques despreciables; no usar silenciosamente en vuelco. Elegir magnitud temporal derivada del ZML requiere definicion visible y decision cientifica; no inventar un indice escalar denominado ZML.
- PLTR Li2019: NO verificado. DOI10.1007/s11771-019-4179-3, https://link.springer.com/article/10.1007/s11771-019-4179-3 y https://journal.hep.com.cn/jocsu/EN/10.1007/s11771-019-4179-3 exponen resumen/preview, no desarrollo completo accesible. Busqueda acotada titulo/DOI/PDF sin manuscrito abierto. Busqueda local nombres Li2019/PLTR/4179 solo encontro nota markdown. No atribuir LTR+dt*dLTR/dt a Li ni asegurar datos suficientes para su predictor. Otro paper abierto mismo autor https://www.extrica.com/article/19579 DOI10.21595/jve.2018.19579 no es sustituto autorizado.

## Mapeo al codigo
viewer/server.py TruckEngine: parametros self.cog,self.inertia,self.centers,self.radius,self.width,self.robot_model.total_mass. assemble_and_transform_robot devuelve CoG y tensor I mundial=R*Ibody*R.T. Bucle de simulacion tiene fuerzas por contacto, puntos de aplicacion, momentos sobre CoG, orientacion y cinematica. Frames aun no publican CoG/inercia/metadatos geometricos.
Omega es mundial (engine_state.py:22-30): transformar R.T*omega para SI. Ruedas tienen 48 muestras cada una, pares de centros +/-Y; contactos de carroceria van despues. Calcular indices al paso fisico 0.01s antes de reducir frames a ~0.04s; PLTR predictivo requerira su propio modelo verificado.

## Comprobacion ejecutada por agente de inspeccion
Importo server con Python existente, instancio y avanzo los tres TruckEngine en malla plana. Salida0, parametros finitos, LTR0 en simetria. No se modificaron fuentes.
MAN5t: m7085kg,Ixx3441kgm2. MAN7t:m8285,Ixx4023. MAN10t:m10785,Ixx5238.
CoGz=.744m, semivia entre centros1.035m, fondo nominal rueda z=-.629m => h1.373m, phi_c37.00985grados, omega_c~1.40519rad/s en los tres (ratios masa/inercia similares).
Estos valores son CHASIS SOLO por decision explicita en server.py:95-97; el motor colapsa geometria de ejes y ruedas sin agregar masas inventadas. Computabilidad del modelo no significa validacion del camion real ni de limites nominales.

## Pendientes materiales
Texto completo Li para verificar PLTR; seleccion/definicion de magnitud temporal de ZML; politica de estados sin apoyo y datos no validos. Mantener graficas sincronizadas con reproduccion, sin streaming, conforme respuesta ronda2. SI energetico unicamente, conforme ronda1.

