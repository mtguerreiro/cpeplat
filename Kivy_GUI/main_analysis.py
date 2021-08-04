
from kivy.app import App
import exp_widgets
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout


import posixpath
import kivy
import csv
import numpy as np
import json
from kivy.app import App
from kivy.uix.popup import Popup
from kivy.core.window import Window
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.checkbox import CheckBox
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.scrollview import ScrollView
from kivy.clock import Clock
from kivy.properties import ObjectProperty, StringProperty, BooleanProperty
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvas
import matplotlib.pyplot as plt
import matplotlib
import time
import os

plt.style.use('seaborn-colorblind')
matplotlib.rcParams['mathtext.fontset'] = 'stix'
matplotlib.rcParams['font.family'] = 'Sans Serif'
matplotlib.rcParams.update({'font.size': 12})
matplotlib.rcParams["figure.dpi"] =300
l_fs = 4.5
title_fs = 5
matplotlib.rcParams['savefig.format'] ='pdf'
matplotlib.rcParams['savefig.dpi'] =600
matplotlib.rcParams['lines.linewidth'] =0.75
matplotlib.rcParams['axes.titlesize'] =title_fs
matplotlib.rcParams['xtick.labelsize'] =l_fs
matplotlib.rcParams['ytick.labelsize'] =l_fs
matplotlib.rcParams['figure.constrained_layout.use'] =True
#matplotlib.rcParams['figure.autolayout'] =True
matplotlib.rcParams['figure.figsize'] =(10,10)
matplotlib.rcParams['figure.titlesize'] ='medium'
matplotlib.rcParams['legend.fontsize'] =4
matplotlib.rcParams['legend.fancybox'] =False
matplotlib.rcParams['legend.framealpha'] =0.5
matplotlib.rcParams['grid.alpha'] =0.5
matplotlib.rcParams['figure.constrained_layout.hspace'] =0.000


 
plt.rc('axes', unicode_minus=False)
class DataObject():
    def __init__(self,file_in=''):
        self.data=list()
        self.scale=1;
        self.filepath=file_in
        self.col_number=0;
        self.extension=''
        self.row_numbers=0
        self.label_std=['t','$V_{i}$','V_{ib}','$V_o$','$V_{ob}$','I_{l}','I_{lavg)','u']

    
    def set_new_data(self,new_data):
        new=[]
        new.append(new_data)
        self.data=new_data[0].copy()
        
    def read_oscilloscope_file(self):
        new_list=[]
        with open(self.filepath, "r") as csvfile:
            reader= csv.reader(csvfile,delimiter=' ')
            for row in reader:
                if (self.row_numbers==0):
                    self.row_numbers=len(row)
                new_row=list()
                for number in row:
                    new_row.append(float(number))
                    
                new_list.append(new_row)
            self.col_number=len(new_row)
        self.data=new_list.copy()

    def read_c2000_csv(self):
        new_list=[]
        with open(self.filepath, "r") as csvfile:
            reader= csv.reader(csvfile,delimiter=',')
            next(reader)
            for row in reader:
                if (self.row_numbers==0):
                    self.row_numbers=len(row)
                new_row=list()
                for number in row:
                    new_row.append(float(number))                    
                new_list.append(new_row)
            
            self.col_number=len(new_row)
        self.data=new_list.copy()
        
    def updateExtension(self):
        self.extension =self.filepath.split('.')[-1]
    def getDataList(self,col_n,scale):
        np_data = (np.array(self.data)*scale)     
        np_columns=np_data[:,col_n]
        columns=np_columns.tolist()
        return columns
    
    def getColNum(self):
      return self.col_number
  

class ChooseInputs(GridLayout):
    def __init__(self,**kwargs):
        super(ChooseInputs,self).__init__(**kwargs)
    
    pass

class InputsHeader(BoxLayout):
    def __init__(self,**kwargs):
        super(InputsHeader,self).__init__(**kwargs)
        self.size_hint_y=0.025
    pass


class FileHeader(BoxLayout):
    def __init__(self,**kwargs):
        super(FileHeader,self).__init__(**kwargs)
        self.spacing=10
        self.pos_hint={'y':0.800,'left':0}
        self.id_num=0
        self.input_rows=list()
        self.input_num=0
        self.add_widget(InputsHeader(),0)
        self.source_path=self.ids.get_file.text
        self.data_obj= DataObject()
        

    
     
    file_path = StringProperty("No file chosen")
    the_popup = ObjectProperty(None)
    
    def getInputs(self):
        return self.input_rows
                        
    def read_file(self): 
        self.data_obj.updateExtension()
        

        if (self.data_obj.extension =='dat'): #add options for when files are different
            try:
                self.data_obj.read_oscilloscope_file()
                for i in range(0,self.data_obj.col_number):
                    self.add_input()
                    

            except ValueError as ve:
                print(ve)
        if (self.data_obj.extension =='csv'): #add options for when files are different
            try:
                self.data_obj.read_c2000_csv()
                for i in range(0,self.data_obj.col_number):
                    self.add_input()    

            except ValueError as ve:
                print(ve)
        
    def add_input(self):
        if self.input_num<self.data_obj.getColNum():      
            self.pos_hint['y']=(self.pos_hint['y']-0.030)
            self.size_hint_y=self.size_hint_y+0.030
            self.input_rows.append(ChooseInputs())
            self.input_rows[self.input_num].id="input_"+str(self.input_num)
            self.input_rows[self.input_num].ids.col_num.text=str(self.input_num+1)
            self.add_widget(self.input_rows[self.input_num],0)
            self.input_num=self.input_num+1

    def load_text_input(self):        
        self.data_obj=DataObject(self.ids.get_file.text)
        self.read_file()

    def getDataListParent(self,index,scale):
        return self.data_obj.getDataList(index,scale)       

    def open_popup(self):
        self.the_popup = FileChoosePopup(load=self.load)
        self.the_popup.open()
    def load(self, selection):
        self.file_path = str(selection[0])  
        self.the_popup.dismiss()
        # check for non-empty list i.e. file selected
        if self.file_path:
            self.ids.get_file.text = self.file_path
        self.data_obj=DataObject(self.file_path)

    def remove(self):
        #self.add_input()
        pass

        
class MyFigure(FigureCanvasKivyAgg):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.size_hint=(0.5, 0.65)
        self.pos_hint={'y': 0.15, 'right':1}
        
        
class DrawPlot(BoxLayout):
    def __init__(self, **kwargs):
        super(DrawPlot, self).__init__(**kwargs)
        self.size_hint=(0.5,0.5)
        self.pos_hint={'x':0.5,'right':1} 
        self.plot_num=4;
        self.title='Experiment Plots'
        self.legends=(['$V_{i}$','$V_{ib}$'], ['$V_o$','$V_{ob}$'], ['$I_l$','$I_{la}$'] ,['$u$'])
        self.labels=list()
        self.y_plot=list()      
        self.test=False
        self.x_labels=''
        self.y_labels=''



        
    def draw_my_plot(self):
        self.clear_widgets()

        fig, ax = plt.subplots(self.plot_num)
        #fig.cla()
        if self.plot_num>1:
            if self.title:
                fig.suptitle(self.title) 
            for i in range(0,self.plot_num):

                ax[i].cla()
                ax[i].plot(self.y_plot[i])
                ax[i].legend(self.legends[i],loc='upper left', ncol=2)
                ax[i].grid()
            canvas =  FigureCanvas(figure=fig)
                
        elif self.plot_num==1:
            try:        
                self.set_title(self.parent.title.text)
                fig.suptitle(self.title,fontsize=22) 
            except Exception as ve:
                print(ve)
            try:        
                ax.set_xlabel(self.parent.x_label.text)
            except Exception as ve:
                print(ve)
            try:        
                ax.set_ylabel(self.parent.y_label.text)
            except Exception as ve:
                print(ve)
                                
            for index,line in enumerate(self.y_plot):
                ax.plot(line)  
                
            ax.grid()
            ax.legend(self.legends,loc='upper left', ncol=2)        
            canvas =  FigureCanvas(figure=fig) 
            
       
        self.add_widget(canvas, 0)
        self.test=True
        return;
    def set_new_data(self,y_plot_new):
        new=[]

        for index, data in enumerate(y_plot_new):
            new.append(data)
            
        self.y_plot=new.copy() 
        #print(self.y_plot)
        return;
    def set_new_legends(self,new_legends):
        self.legends=new_legends
        return;
    def set_plot_num(self,num):
        self.plot_num=num
        return;
    def set_title(self,title):
        self.title=title
        return;
    def set_xlabels(self,labels):
        self.x_labels=labels
        return;
    def set_ylabels(self,labels):
        self.x_labels=labels
        return;
        
class FileChoosePopup(Popup):
    load = ObjectProperty()
    
class Analysis(BoxLayout):
    title=ObjectProperty(None)
    x_label=ObjectProperty(None)
    y_label=ObjectProperty(None)
    ini=BooleanProperty(False)
    source_forms=ObjectProperty(None)
    the_popup=ObjectProperty(None)
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.source_forms=[]
        self.source_num=0
        self.y_init=.920
        self.ini=True  
        self.new_fig=MyFigure(figure=plt.gcf())
        self.new_fig.id="graph"
        self.event=Clock.schedule_once(lambda dt: self.add_source(), 1)
        self.event()


    def save_config(self):
        config_list = list()
        labels=list()
        scales=list()
        check_status=list()
        
        
        for source in self.source_forms:  
            path=source.ids.get_file.text
            if path != '':
                labels=[]
                scales=[]
                check_status=[]

                for input_line in source.input_rows:
                    labels.append(input_line.ids.label_input.text)
                    scales.append(input_line.ids.scale_input.text)
                    check_status.append(input_line.ids.checkbox_show.active)

                config_list.append([path,labels,scales,check_status])
        print(config_list)
        with open("config.json", "w") as outfile:
             json.dump(config_list, outfile)

    def load_config(self):
        with open("config.json", "r") as infile:
            config_list=json.load(infile)
        print(config_list)

        """add new sources and lines"""
        for s_ind, line in enumerate(config_list, start=0):
            if s_ind >=len(self.source_forms):
                print("adding source: ",s_ind)
                self.add_source()
            self.source_forms[s_ind].ids.get_file.text=line[0]
            self.source_forms[s_ind].load_text_input()
            for l_ind, in_row in enumerate(self.source_forms[s_ind].input_rows):
                print(l_ind)
                try:
                    in_row.ids.label_input.text=line[1][l_ind]
                    in_row.ids.scale_input.text=line[2][l_ind]
                    in_row.ids.checkbox_show.active=line[3][l_ind]
                    print("config.json info loaded sucessfully")
                except Exception as ve:
                    print(ve)
        self.update_graphs()
                
    def add_source(self):

        self.source_forms.append(FileHeader())
        self.source_forms[self.source_num].pos_hint={'y':self.y_init-(self.source_num*0.15),'left':0}
        self.source_forms[self.source_num].id="src_"+str(self.source_num)
        self.source_forms[self.source_num].ids.src_n.text="Source "+str(self.source_num+1)+":"
        self.ids.form_layout.add_widget(self.source_forms[self.source_num],0)
        self.source_num+=1
        
    
    def remove_source(self):
        if(self.source_num)>1:
            try:
                self.ids.form_layout.remove_widget(self.source_forms[self.source_num-1])
                self.source_num-=1
            except:
                pass
                                           
    def update_sources(self):
        
        for source, ind in enumerate(self.source_forms):
                if (source.ids.get_file.text != ''):
                    source.data_list.append(DataObject(source.ids.getfile.text))
                    
 
    def open_popup(self):
        process = Popen(['python3', 'main_analysis.py'], stdout=PIPE, stderr=PIPE)
                
    def update_graphs(self):
        try:
            if not (self.ini):
                self.ids.form_layout.remove_widget(self.new_fig)
                plt.cla()
        except:
            pass
        x_data=list()
        legends=list()
       # legends.append('   ')
        for source in self.source_forms:
            for index,row in enumerate(source.getInputs()):
                scale=float(row.ids.scale_input.text)
                if (row.ids.checkbox_show._get_active()):
                    x_data.append(source.getDataListParent(index,scale))
                    legends.append(row.ids.label_input.text)
        self.ids.plot_al.set_plot_num(1)
        self.ids.plot_al.set_new_legends(legends)
        self.ids.plot_al.set_new_data(x_data)
        self.ids.plot_al.draw_my_plot()
        self.ini=False
    
    def export_graph(self):
        """save graph to png"""
        timestr = time.strftime("%Y%m%d-%H%M%S")
        #filepath="./Graphs/"+"Buck_"+self.timestr+".png"
        filename="Analysis"+str(timestr)+".png"
        try:
            self.ids.plot_al.export_to_png(filename)
            source ="./"+filename
            destination="./Images/Analysis/"+filename
            os.replace(source,destination)
            print("file saved sucessfully")

        except Exception as ve:
            print(ve)

        pass   
    pass

    
        
    
    
class AnalysisApp(App):
    def build(self):
        return Analysis()
    
if __name__ == '__main__':
    AnalysisApp().run()
 