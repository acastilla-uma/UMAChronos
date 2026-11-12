from pathlib import Path
from html.parser import HTMLParser
import json, re, textwrap, html, math
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'exports'
class Node:
    def __init__(self,tag='',attrs=()): self.tag=tag; self.attrs=dict(attrs); self.children=[]
    def text(self): return ''.join(c.text() if isinstance(c,Node) else c for c in self.children)
    def find(self,tag):
        return [c for c in self.children if isinstance(c,Node) and c.tag==tag]+[n for c in self.children if isinstance(c,Node) for n in c.find(tag)]
class Parser(HTMLParser):
    def __init__(self): super().__init__();self.root=Node();self.stack=[self.root]
    def handle_starttag(self,tag,attrs):
        n=Node(tag,attrs);self.stack[-1].children.append(n)
        if tag not in ('input','meta','link','br','img','hr'): self.stack.append(n)
    def handle_endtag(self,tag):
        for i in range(len(self.stack)-1,0,-1):
            if self.stack[i].tag==tag: self.stack=self.stack[:i];break
    def handle_data(self,data): self.stack[-1].children.append(data)
p=Parser(); source=(ROOT/'index.html').read_text(encoding='utf-8');p.feed(source)
elements=[]; panel_boxes=[]; ids=set(); sources=set(); group=''
FG='#e6eef3'; MUTED='#a6babc'; EDGE='#43595d'; PANEL='#17262c'; BG='#0d161c'; MINT='#43ddbd'
def base(kind,x,y,w,h,stroke=FG,fill='transparent'):
    id=f'ff-{len(elements)+1:05d}'
    e=dict(id=id,type=kind,x=x,y=y,width=w,height=h,angle=0,strokeColor=stroke,backgroundColor=fill,fillStyle='solid',strokeWidth=1,strokeStyle='solid',roughness=0,opacity=100,groupIds=[group] if group else [],frameId=None,roundness=None,seed=len(elements)+1,version=1,versionNonce=len(elements)+1,isDeleted=False,boundElements=None,updated=1,link=None,locked=False)
    elements.append(e);return e

def box(x,y,w,h,fill=PANEL,stroke=EDGE):return base('rectangle',x,y,w,h,stroke,fill)
def text(x,y,value,size=16,color=FG,width=None):
    value=re.sub(r'\s+',' ',str(value)).strip()
    lines=(textwrap.wrap(value,width=max(10,int(width/(size*.54)))) if width else [value]) or [' ']
    t='\n'.join(lines) or ' '
    w=min(width or max(len(l) for l in lines)*size*.56,max(len(l) for l in lines)*size*.56)
    e=base('text',x,y,w,len(lines)*size*1.35,color)
    e.update(text=t,originalText=t,fontSize=size,fontFamily=2,textAlign='left',verticalAlign='top',containerId=None,autoResize=True,lineHeight=1.35)
    return e['height']
def button(x,y,label,primary=False,w=None):
    w=w or len(label)*8+30;box(x,y,w,38,MINT if primary else '#22343c');text(x+12,y+9,label,14,'#082c23' if primary else FG);return w

def field(x,y,label,value='',w=245):
    h=text(x,y,label,13,MUTED,w);box(x,y+h+8,w,37,'#0f1e22');text(x+10,y+h+17,value or ' ',14,MUTED,w-20);return h+58

def panel(x,y,title,width=850):
    global group
    group='panel-'+str(len(panel_boxes));rect=box(x,y,width,100);panel_boxes.append(rect);text(x+22,y+18,title,21);button(x+width-56,y+10,'×',w=36)
    return rect,y+70

def finish(rect,end):rect['height']=end-rect['y']+26
specs={
'pose':[('X · m',0),('Y · m',0),('Z · m',1.2),('Balanceo · °',0),('Cabeceo · °',0),('Guiñada · °',0)],
'velocity':[('Vx · m/s',0),('Vy · m/s',0),('Vz · m/s',0),('ωx · rad/s',0),('ωy · rad/s',0),('ωz · rad/s',0)],
'physics':[('Velocidad tangencial · m/s',0),('Dirección · °',0),('Fricción general · μ',.7),('Rigidez general · N/m',1000000)],
 'timing':[('Paso · s',.002),('Duración · s',2)]}
classes={'desconocido':(.7,1e6),'grava':(.65,8e5),'roca':(.85,2e6),'plantas':(.45,3e5),'barro':(.25,1e5),'tierra':(.6,6e5)}
metrics=['Tiempo','Posición · m','Velocidad · m/s','Aceleración · m/s²','Aceleración angular · rad/s²','Par total · N·m','Objetivo · m/s','Error de seguimiento XYZ · m','Error de velocidad · m/s','Aceleración tangencial · m/s²','Error de aceleración · m/s²']

def render(n,x,y,w=806):
    if not isinstance(n,Node):return y
    id=n.attrs.get('id')
    if id:sources.add(id)
    if n.tag=='p':return y+text(x,y,n.text(),14,MUTED,w)+18
    if n.tag=='h3':return y+text(x,y,n.text(),17)+18
    if n.tag=='label':
        label=''.join(c for c in n.children if isinstance(c,str)).strip(); inputs=n.find('input');sels=n.find('select')
        if inputs and inputs[0].attrs.get('type')=='checkbox':
            a=inputs[0].attrs;box(x,y,18,18,'#0f1e22');text(x+2,y-1,'✓' if 'checked' in a else '',15,MINT);return y+text(x+30,y,label,14,FG,w-30)+18
        a=(inputs or sels)[0].attrs if inputs or sels else {}
        val=a.get('value') or a.get('placeholder','')
        if sels:val='MAN 10t · rígido experimental  ▾' if a.get('id')=='vehicle' else 'desconocido  ▾'
        if a.get('type')=='file':val='Elegir archivo     Ningún archivo seleccionado'
        return y+field(x,y,label,val,w)+12
    if id=='vehicleLimitations':
        return y+text(x,y,'Masa e inercia del chasis; suspensión fija, sin motor, transmisión ni neumáticos deformables. No validado como vehículo completo.',14,MUTED,w)+18
    if n.tag=='details':
        return y+text(x,y,'▸ '+n.find('summary')[0].text()+' (inventario completo en los paneles inferiores)',14,MUTED,w)+20
    if n.tag=='canvas':
        box(x,y,w,260,'#101e22');text(x+20,y+110,'Mapa XY · disponible al importar la nube PLY',15,MUTED);return y+278
    if n.tag=='table':
        heads=[a.text() for a in n.find('th')];text(x,y,'   |   '.join(heads),14);y+=34
        if any(a.attrs.get('id')=='classRows' for a in n.find('tbody')):
            for name,(mu,k) in classes.items():
                text(x,y+10,name,14);field(x+260,y,'',str(mu),200);field(x+490,y,'',str(int(k)),245);y+=65
        else:text(x,y,'Sin puntos de trayectoria · añade desde el mapa o desde la pose.',14,MUTED,w);y+=40
        return y+15
    if id in specs:
        fields=specs[id]
        for i,(label,v) in enumerate(fields):field(x+(i%3)*270,y+(i//3)*84,label,str(v),250)
        return y+math.ceil(len(fields)/3)*84+12
    if id=='terrainLegend':
        text(x,y,'   '.join(classes),14,MINT,w);return y+42
    if id=='metrics':
        text(x,y,'Campos disponibles tras calcular la simulación',13,MINT);y+=30
        for label in metrics:text(x,y,label,14);text(x+640,y,'—',14,MUTED);y+=33
        return y+15
    if id=='selection':return y+text(x,y,n.text(),14,MUTED,w)+16
    if n.attrs.get('class')=='actions':
        buttons=n.find('button')+n.find('a');cx=x
        for b in buttons:
            width=len(b.text())*8+30
            if cx+width>x+w:y+=48;cx=x
            button(cx,y,b.text(),'primary' in b.attrs.get('class',''),width);cx+=width+10
        return y+56
    for c in n.children:y=render(c,x,y,w)
    return y

# Main screen: all persistent controls, with the application's empty scene.
group='principal';box(0,0,1400,850,BG);box(0,0,1400,70,'#101d24');text(22,20,'◈ FusionForce',25)
cx=240
for title in ['Proyecto','Nube','Vehículo','Trayectoria','Terreno','Configuración','Resultados']:
    text(cx,26,title,14,MUTED);cx+=len(title)*8+24
text(1100,10,'MODELO EXPERIMENTAL',11,'#f1c676');button(1240,18,'▶ Simular',True,135)
box(24,92,610,118,'#101e24');text(40,108,'Carga una nube PLY para comenzar',16,MINT);text(40,138,'Sin terreno cargado',14);text(40,166,'Mapa completo · nube cargada en memoria · arrastra para orbitar',14,MUTED)
box(1130,92,245,155,'#101e24');text(1146,108,'Vehículo',17);text(1146,139,'En espera',14);text(1146,166,'Haz clic en un punto blanco para inspeccionar sus fuerzas.',12,MUTED,210);button(1146,204,'Ver resultados')
for i in range(12):
    e=base('line',220+i*80,360,250,280,'#223536');e.update(points=[[0,0],[250,280]],lastCommittedPoint=None,startBinding=None,endBinding=None,startArrowhead=None,endArrowhead=None)
for i in range(9):
    e=base('line',220,360+i*30,880,0,'#223536');e.update(points=[[0,0],[880,0]],lastCommittedPoint=None,startBinding=None,endBinding=None,startArrowhead=None,endArrowhead=None)
text(28,728,'● Normal     ● Fricción     ● Momento     ● Ruta objetivo     ● Trayectoria real     ● Terreno desconocido',13,MUTED)
box(24,770,1352,58,'#101e24');button(38,780,'▶',w=40);box(96,797,1080,4,MINT,MINT);text(1190,789,'0,00 s',15);button(1265,780,'↺',w=40);button(1317,780,'⌖',w=40)
text(0,-68,'FusionForce · interfaz completa',32);text(0,-28,'Elementos editables · menús desplegados por separado · valores iniciales y campos de resultados',16,MUTED)
dialogs=p.root.find('dialog'); placements=[(0,1000),(930,1000),(1860,1000),(0,2600),(930,2600),(1860,2600),(0,4200)]
for n,(x,y) in zip(dialogs,placements):
    rect,cy=panel(x,y,n.find('h2')[0].text());body=next(c for c in n.children if isinstance(c,Node) and c.attrs.get('class')=='body')
    cy=render(body,x+22,cy);finish(rect,cy)
# Expanded model selector, including all compatible options.
rect,cy=panel(930,4200,'Vehículo · selector de modelo desplegado')
for name in ('MAN 5t · rígido experimental','MAN 7t · rígido experimental','MAN 10t · rígido experimental'):
    box(952,cy,806,42,'#0f1e22');text(966,cy+10,name,15);cy+=48
cy+=text(952,cy+12,'Opciones disponibles en el selector Modelo compatible.',14,MUTED,806)+24
finish(rect,cy)
# Full expanded inventory, split into four readable columns.
catalog=json.loads((ROOT/'catalog.json').read_text(encoding='utf-8'))
for col in range(4):
    x=col*930;y=5550;rows=catalog['vehicles'][col*20:(col+1)*20]
    rect,cy=panel(x,y,f'Inventario Chrono · {col*20+1}–{col*20+len(rows)} de 80')
    cy+=text(x+22,cy,'Revisión: '+catalog['source']['commit'],12,MUTED,806)+20
    for row in rows:
        cy+=text(x+22,cy,row.get('name',row['id']),17,FG,806)+5
        cy+=text(x+22,cy,row['status'],13,MINT,806)+5
        cy+=text(x+22,cy,row.get('reason') or row.get('limitations',''),13,MUTED,806)+25
    finish(rect,cy)

result=dict(type='excalidraw',version=2,source='FusionForce',elements=elements,appState=dict(viewBackgroundColor=BG,gridSize=None,scrollX=60,scrollY=100,zoom={'value':.4}),files={})
out=OUT/'FusionForce-interfaz-completa.excalidraw';out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
# Preview of the editable boards, rendered directly from their native elements.
minx=-40;miny=-90;maxx=max(e['x']+e['width'] for e in elements)+40;maxy=max(e['y']+e['height'] for e in elements)+40
scale=.45;im=Image.new('RGB',(int((maxx-minx)*scale),int((maxy-miny)*scale)),BG);draw=ImageDraw.Draw(im)
for e in elements:
    x=(e['x']-minx)*scale;y=(e['y']-miny)*scale;w=e['width']*scale;h=e['height']*scale
    if e['type']=='rectangle':draw.rectangle((x,y,x+w,y+h),fill=None if e['backgroundColor']=='transparent' else e['backgroundColor'],outline=e['strokeColor'],width=1)
    elif e['type']=='line':draw.line([(x+px*scale,y+py*scale) for px,py in e['points']],fill=e['strokeColor'],width=1)
    elif e['type']=='text':
        font=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',max(7,round(e['fontSize']*scale)))
        draw.multiline_text((x,y),e['text'],font=font,fill=e['strokeColor'],spacing=round(e['fontSize']*.35*scale))
im.save(OUT/'FusionForce-interfaz-completa.png')
# Check artifact structure, unique IDs, all menus, editable content, and non-overlapping panels.
assert len({e['id'] for e in elements})==len(elements)
assert len(dialogs)==7
for n in dialogs:assert any(e.get('text')==n.find('h2')[0].text() for e in elements)
assert all(e['type'] in ('text','rectangle','line') and e['width']>=0 and e['height']>=0 for e in elements)
for i,a in enumerate(panel_boxes):
    for b in panel_boxes[i+1:]:
        assert a['x']+a['width']<=b['x'] or b['x']+b['width']<=a['x'] or a['y']+a['height']<=b['y'] or b['y']+b['height']<=a['y'],(a,b)
print(json.dumps({'file':str(out),'elements':len(elements),'menus':len(dialogs),'inventory_rows':len(catalog['vehicles']),'preview':im.size,'verified':True},ensure_ascii=False))
