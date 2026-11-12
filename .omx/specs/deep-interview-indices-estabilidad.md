# Panel de estabilidad — especificacion cerrada
Fecha2026-11-06. Perfil standard. 4 rondas. Ambiguedad final0.10. Proyecto existente. Contexto: .omx/context/indices-estabilidad-20261106T153043Z.md. Evidencia cientifica: .omx/research/viabilidad-indices-estabilidad.md.

## Intencion y resultado
Comparar y entender LTR/PLTR, ZML y SI energetico durante el movimiento reproducido. Panel persistente arriba derecha, graficas progresivas sincronizadas con avanzar/pausar/retroceder. Debajo de las graficas: valores actuales, todos los parametros usados, unidades y formulas. Estado vacio explicito antes de resultados.

## Decisiones del usuario
1 SI energetico unicamente (umbral de masa/geometria/inercia del vehiculo activo).
2 Graficas durante reproduccion; conservar calculo completo previo, sin streaming.
3 Comprobar viabilidad antes de decidir aproximaciones: investigacion realizada, pruebas en 3 MAN.
4 PLTR cinematico inspirado en Li permitido, formula e hipotesis visibles. No presentarlo como ecuacion10 literal.

## Alcance y limites
Calculo/serializacion de indicadores y panel. No cambiar dinamica, controles, modelo de neumaticos ni suspension. Sin dependencias nuevas; no modificar Chrono-docker ni fuentes bibliograficas. No calibrar kf/kr ni copiar constantes SUV/camion completo. Mantener proyectos y CSV existentes compatibles. Nuevos resultados llevan parametros en sus frames y quedan guardados en proyectos; proyectos antiguos muestran ausencia de indicadores, nunca ceros inventados. No controles de seguridad ni alertas con umbrales no validados.
Decisiones delegadas por contexto: detalles de presentacion/accesibilidad, umbral numerico de degeneracion, pruebas y estructura interna. Horizonte nominal PLTR0.5s, visible como parametro e hipotesis de extrapolacion local de primer orden; calculo causal sin futuro de reproduccion. ZML se representa con dos coordenadas de interseccion de su linea con un plano horizontal de referencia, NO inventar un indice normalizado. Referencia a altura media ponderada por carga de contactos portantes; mostrar z_ref y origen/marco. Esta representacion no reemplaza el apoyo 3D por una prueba de estabilidad plana. Sin apoyo o datos degenerados: valor no disponible con razon, nunca cero seguro.

## Formulas y contexto tecnico
LTR firmado=(FzR-FzL)/(FzR+FzL), cargas verticales mundiales solo de ruedas, lados por identidad local, no Y mundial. Resto de contactos carroceria excluidos de LTR pero incluidos en wrench completo ZML.
PLTR_cin=2h/tw*(ay/g+phi)+2h/(tw*g)*(d(ay_body)/dt+g*dphi/dt)*Dt. phi en radianes, derivados causales al paso fisico, transformacion correcta world->body. Modelo de primer orden, pequeno angulo, geometria nominal constante. No predice terreno ni maniobras futuras. No es extrapolacion del LTR de fuerzas ni eq10 exacta. Primera muestra sin derivada: no disponible.
SI=1-1.15*abs(phi)/phi_c-2.05*(omega_x_body/omega_c)^2; phi_c=atan(b/h),d=hypot(b,h),omega_c=sqrt(2mg(d-h)/(Ixx_cg+md²)). Mantener negativos; umbrales/coefs nominales sin validacion, constantes del vehiculo simulado (chasis rigido).
ZML/COP: fuerza F y momento M respecto CoG en marco rumbo X adelante,Yizq,Zvertical. x(z)=(-My+zFx)/Fz,y(z)=(Mx+zFy)/Fz. Mostrar x(z_ref),y(z_ref) como dos curvas [m], parametros de linea/wrench y residual torsional M.F/Fz. Es linea COP que corresponde ZML bajo condiciones del paper; residual y limitaciones visibles, sin simplificacion basada solo aceleracion ni margen de seguridad inventado. Fz~0 => no disponible.
Motor publica omega mundial; SI usa R.T*omega. Masa/inercia chasis-only. Puntos de contacto son muestras de contacto compliant. Calcular cada paso fisico0.01 antes de stride visual0.04; no alterar fuerzas/estado. Referencias: Li PDFpagina8 eq10, Brecelj paginas6-9 eq8-21, Chrono stability.py.

## Contrato de implementacion
viewer/stability.py: numpy independiente, StabilityMonitor(mass,inertia_body,cog_body,wheel_centers,tire_radius,wheel_sample_count, horizon=.5, gravity=9.81). update(time,points,forces,cog,orientation,velocity,acceleration,angular_velocity) => dict de entradas ltr,pltr,zml,si. Cada entrada {value:float|None (zml:[x,y]|None),params:{nombre:numero|None},reason:str|None}. Parametros estables con unidades documentadas; sin NaN/Inf. Estado de derivadas por corrida.
Servidor integra una instancia por simulacion, una llamada por paso usando fuerzas ya existentes. Guarda salida en frame.stability. Frontend usa frame.stability, no recalcula desde muestras reducidas.
Panel 3 graficas: LTR+PLTR (dos curvas adimensionales), ZML (x/y [m]), SI energetico. Formulas y parametros debajo, sin informacion futura al retroceder. Graficas Canvas nativas, sin nuevas dependencias. Series no conectan huecos de valores no disponibles. Panel arriba derecha tambien en movil, scroll interno sin tapar controles inferiores.

## Aceptacion y pruebas
- Graficas y valores actuales coinciden con frame seleccionado; pausa estable y retroceso corta curvas futuras.
- Parametros y formulas legibles con unidades, valores negativos no recortados, sin NaN.
- Vacio, frames antiguos, aire/Fz0, primera derivada, pitch singular y datos invalidos claramente no disponibles.
- LTR simetria0/asimetria signo, agrupacion invariante ante yaw y exclusión chasis.
- PLTR con aceleracion lateral lineal y roll conocido tiene prediccion analitica; no fuga de datos futuros.
- SI identidad/yaw equivalente y parametros propios MAN; ZML fuerzas puramente verticales (linea vertical), wrench inclinado y cambio origen coherente.
- Pruebas unitarias numpy y pruebas de integracion servidor/frame; pruebas JS reales de eventos de reproduccion/invalidacion/reapertura, sintaxis, suite existente. Comprobacion visual real si navegador local disponible; documentar gap si no.

## Presion y cierre
Se cuestiono asumir datos suficientes para predictor Li: usuario exigio investigacion; PDFs revelaron parametros ausentes. Usuario eligio explicitamente variante cinematica. Non-goals cerrados por alcance de panel y conservacion simulacion; ninguna ampliacion de fisica autorizada. Otra pregunta sobre estilo/horizonte inicial no cambia el alcance; decisiones numericas visibles y documentadas. Se cierra entrevista y se transfiere a ejecucion local autorizada para terminar peticion original; sin activar runtime tmux ni crear objetivos nativos no solicitados.
