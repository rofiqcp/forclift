#!/usr/bin/env python3
import argparse, json, os, re, shutil, struct, subprocess, zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

ROOT = Path(os.environ.get('AGV_WS') or os.environ.get('AGV_ROOT') or '/home/otomasi2/forclift')
MAP_DIR = ROOT / 'src/navigation/maps'
BUILD_DIR = MAP_DIR / 'build_map'
SELECTED_PGM = MAP_DIR / 'navigation_selected.pgm'
SELECTED_YAML = MAP_DIR / 'navigation_selected.yaml'
SELECTED_NAME = MAP_DIR / 'navigation_selected_name.txt'

def safe_name(value):
    return re.sub(r'[^A-Za-z0-9_-]+', '_', str(value or '').strip()).strip('_')[:64]

def yaml_meta(path):
    try: text = Path(path).read_text(encoding='utf-8', errors='replace')
    except OSError: return {}
    out = {}
    m = re.search(r'^\s*resolution\s*:\s*([-+0-9.eE]+)', text, re.M)
    if m: out['resolution'] = float(m.group(1))
    m = re.search(r'^\s*origin\s*:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)', text, re.M)
    if m: out.update(origin_x=float(m.group(1)), origin_y=float(m.group(2)))
    return out

def read_pgm(path):
    data = Path(path).read_bytes(); n = len(data); i = 0
    def token():
        nonlocal i
        while i < n:
            if data[i] == 35:
                while i < n and data[i] not in (10, 13): i += 1
            elif chr(data[i]).isspace(): i += 1
            else: break
        j = i
        while i < n and not chr(data[i]).isspace() and data[i] != 35: i += 1
        return data[j:i]
    magic = token(); w = int(token()); h = int(token()); maxv = int(token())
    if magic == b'P5':
        if i < n and data[i] == 13: i += 1
        if i < n and data[i] == 10: i += 1
        elif i < n and chr(data[i]).isspace(): i += 1
        pix = data[i:i+w*h]
        if len(pix) != w*h: raise ValueError('PGM payload pendek')
        if maxv != 255: pix = bytes(min(255, round(v*255/maxv)) for v in pix)
    elif magic == b'P2':
        vals = [int(token()) for _ in range(w*h)]
        pix = bytes(min(255, round(v*255/maxv)) for v in vals)
    else: raise ValueError('PGM format tidak didukung')
    return w, h, pix

def png_bytes(path):
    w,h,pix = read_pgm(path)
    raw = b''.join(b'\x00' + pix[y*w:(y+1)*w] for y in range(h))
    def chunk(kind, payload):
        return struct.pack('>I',len(payload))+kind+payload+struct.pack('>I',zlib.crc32(kind+payload)&0xffffffff)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(raw,6))+chunk(b'IEND',b'')

def map_item(key,name,label,source,pgm,yaml,preview):
    pgm,yaml=Path(pgm),Path(yaml); ok=pgm.is_file() and yaml.is_file()
    out={'key':key,'name':name,'label':label,'source':source,'available':ok,'preview_url':preview}
    if ok:
        try:
            w,h,_=read_pgm(pgm); out.update(width=w,height=h)
        except Exception: pass
        out.update(yaml_meta(yaml))
        out['modified_at_ms']=int(max(pgm.stat().st_mtime,yaml.stat().st_mtime)*1000)
    return out

def call_trigger(slot):
    cmd=['ros2','service','call',f'/navigation/map/start_{slot}','std_srvs/srv/Trigger','{}']
    r=subprocess.run(cmd,cwd=str(ROOT),env=os.environ.copy(),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=12)
    return r.returncode==0, r.stdout.strip()[-600:]

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): return
    def headers_common(self, code, ctype='application/json'):
        self.send_response(code); self.send_header('Content-Type',ctype); self.send_header('Access-Control-Allow-Origin','*'); self.send_header('Access-Control-Allow-Headers','Content-Type'); self.send_header('Access-Control-Allow-Methods','GET,POST,OPTIONS'); self.send_header('Cache-Control','no-store'); self.end_headers()
    def send_json(self, code, obj):
        body=json.dumps(obj,separators=(',',':')).encode(); self.send_response(code); self.send_header('Content-Type','application/json'); self.send_header('Content-Length',str(len(body))); self.send_header('Access-Control-Allow-Origin','*'); self.send_header('Access-Control-Allow-Headers','Content-Type'); self.send_header('Access-Control-Allow-Methods','GET,POST,OPTIONS'); self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(body)
    def do_OPTIONS(self): self.headers_common(204)
    def library(self):
        host=self.headers.get('Host','127.0.0.1:5016'); base=f'http://{host}'
        maps=[map_item('default','map_Navigation','Navigation Map (Default)','default',MAP_DIR/'map_Navigation.pgm',MAP_DIR/'map_Navigation.yaml',base+'/preview/default.png')]
        if BUILD_DIR.is_dir():
            for y in sorted(BUILD_DIR.glob('*.yaml')):
                name=y.stem
                if name=='build_map_latest': continue
                p=BUILD_DIR/(name+'.pgm')
                if p.is_file(): maps.append(map_item('build:'+name,name,name,'build',p,y,base+'/preview/build/'+name+'.png'))
        self.send_json(200,{'ok':True,'maps':maps,'count':len(maps)})
    def do_GET(self):
        path=urlparse(self.path).path
        if path.startswith('/preview/'):
            parts=path.strip('/').split('/')
            pgm=MAP_DIR/'map_Navigation.pgm' if parts[1]=='default.png' else (BUILD_DIR/(safe_name(unquote(parts[-1]).removesuffix('.png'))+'.pgm') if len(parts)>=3 and parts[1]=='build' else None)
            if not pgm or not pgm.is_file(): return self.send_json(404,{'ok':False,'message':'preview tidak ditemukan'})
            try: body=png_bytes(pgm)
            except Exception as e: return self.send_json(500,{'ok':False,'message':str(e)})
            self.send_response(200); self.send_header('Content-Type','image/png'); self.send_header('Content-Length',str(len(body))); self.send_header('Access-Control-Allow-Origin','*'); self.send_header('Cache-Control','no-cache'); self.end_headers(); self.wfile.write(body); return
        self.send_json(404,{'ok':False,'message':'not found'})
    def do_POST(self):
        path=urlparse(self.path).path
        if path=='/api/navigation/map-library': return self.library()
        if path!='/api/navigation/map-use': return self.send_json(404,{'ok':False,'message':'not found'})
        try: body=json.loads(self.rfile.read(int(self.headers.get('Content-Length','0') or 0)) or b'{}')
        except Exception: return self.send_json(400,{'ok':False,'message':'JSON tidak valid'})
        source=str(body.get('source','')).lower()
        if source=='default':
            ok,msg=call_trigger(4); return self.send_json(200 if ok else 409,{'ok':ok,'message':msg or 'Navigation Map (Default)'})
        if source!='build': return self.send_json(400,{'ok':False,'message':'source harus default atau build'})
        name=safe_name(body.get('name'))
        src_pgm,src_yaml=BUILD_DIR/(name+'.pgm'),BUILD_DIR/(name+'.yaml')
        if not name or not src_pgm.is_file() or not src_yaml.is_file(): return self.send_json(404,{'ok':False,'message':'Build Map tidak ditemukan'})
        MAP_DIR.mkdir(parents=True,exist_ok=True)
        tmp=SELECTED_PGM.with_suffix('.pgm.tmp'); shutil.copyfile(src_pgm,tmp); os.replace(tmp,SELECTED_PGM)
        text=src_yaml.read_text(encoding='utf-8',errors='replace'); text=re.sub(r'(?m)^image:\s*.*$','image: navigation_selected.pgm',text)
        tmpy=SELECTED_YAML.with_suffix('.yaml.tmp'); tmpy.write_text(text,encoding='utf-8'); os.replace(tmpy,SELECTED_YAML)
        SELECTED_NAME.write_text(name+'\n',encoding='utf-8')
        ok,msg=call_trigger(5); self.send_json(200 if ok else 409,{'ok':ok,'message':("USE Build Map '%s' • %s"%(name,msg)).strip(),'name':name})

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--host',default='127.0.0.1'); ap.add_argument('--port',type=int,default=5016); a=ap.parse_args()
    print(f'[NAV-MAP-LIBRARY] http://{a.host}:{a.port}',flush=True); ThreadingHTTPServer((a.host,a.port),Handler).serve_forever()
if __name__=='__main__': main()
