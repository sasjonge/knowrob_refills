#!/usr/bin/python
# coding: utf8

from googletrans import Translator
import inflect

import pickle
from shutil import copyfile
import sys, os

class OWLResource:
  """
  bla bla bla
  """
  def __init__(self, name, namespace):
    self.name = name
    self.labels = set()
    self.dataValues = []
    self.objectValues = []
    self.namespace = namespace
    self.types = set()

  def has_type(self,typeName):
    if typeName==self.name or typeName=='' or typeName==None: return
    self.types.add(typeName)

  def add_label(self,label):
    self.labels.add(label)
  
  def has_data_value(self,relation,data_type,data_value):
    self.dataValues.append((relation,data_type,data_value))
  
  def has_object_value(self,relation,object_value):
    self.objectValues.append((relation,object_value))

  def write_plain(self, f):
    f.write(str(self)+'\n')
    for y in self.labels:               f.write('  label '+y+'\n')
    for y in self.types:                f.write('  type  '+y+'\n')
    for (rel,t,val) in self.dataValues: f.write('  '+rel+' '+val+'\n')
    for (rel,t,val) in self.dataValues: f.write('  '+rel+' '+val+'\n')
  
  def __repr__(self):
    return self.name

############################################
############ OWL Classes
############################################

class OWLClass(OWLResource):
  """
  bla bla bla
  """
  def __init__(self, className, namespace):
    OWLResource.__init__(self,className,namespace)
  
  def write_owl(self, f):
    types = list(self.types)
    types.sort()
    #f.write('    <!-- "'+self.namespaceIRI+self.name+'" -->\n\n')
    f.write('\n    <owl:Class rdf:about="&'+self.namespace+';'+self.name+'">\n')
    #for l in self.labels:
    #  f.write('        <rdfs:comment rdf:datatype="&xsd;string">'+l+'</rdfs:comment>\n')
    for y in types:
      # TODO: avoid redundant type statements: check if any other type is subclass of y
      f.write('        <rdfs:subClassOf rdf:resource="&'+self.namespace+';'+y+'"/>\n')
    for (role,t,value) in self.dataValues:
      f.write('        <rdfs:subClassOf>\n')
      f.write('            <owl:Restriction>\n')
      f.write('                <owl:onProperty rdf:resource="&'+self.namespace+';'+role+'"/>\n')
      f.write('                <owl:hasValue rdf:datatype="'+t+'">'+str(value)+'</owl:hasValue>\n')
      f.write('            </owl:Restriction>\n')
      f.write('        </rdfs:subClassOf>\n')
    for (role,value) in self.objectValues:
      f.write('        <rdfs:subClassOf>\n')
      f.write('            <owl:Restriction>\n')
      f.write('                <owl:onProperty rdf:resource="&'+self.namespace+';'+str(role)+'"/>\n')
      f.write('                <owl:hasValue rdf:resource="&'+self.namespace+';'+str(value)+'"/>\n')
      f.write('            </owl:Restriction>\n')
      f.write('        </rdfs:subClassOf>\n')
    f.write('    </owl:Class>\n')
  
  def __repr__(self):
    return self.name

############################################
############ OWL Individuals
############################################

class OWLIndividual(OWLResource):
  """
  bla bla bla
  """
  def __init__(self, instanceName, namespace):
    OWLResource.__init__(self,instanceName,namespace)
  
  def write_owl(self, f):
    types = list(self.types)
    types.sort()
    #f.write('    <!-- "'+self.prefix+self.name+'" -->\n\n')
    f.write('\n    <owl:NamedIndividual rdf:about="&'+self.namespace+';'+self.name+'">\n')
    #for l in self.labels:
    #  f.write('        <rdfs:comment rdf:datatype="&xsd;string">'+l+'</rdfs:comment>\n')
    for y in types:
      # TODO: avoid redundant type statements: check if any other type is subclass of y
      f.write('        <rdf:type rdf:resource="&'+self.namespace+';'+y+'"/>\n')
    for (rel,t,val) in self.dataValues:
      f.write('        <shop:'+rel+' rdf:datatype="'+t+'">'+val+'</shop:'+rel+'>\n')
    for (rel,val) in self.objectValues:
      f.write('        <shop:'+rel+' rdf:resource="'+val+'"/>\n')
    f.write('    </owl:NamedIndividual>\n')
  
  def __repr__(self):
    return self.name

############################################
############ Resource manager
############################################

class LabelTranslator:
  """
  bla bla bla
  """
  # auto save such that terminatingthe  script won't wipe the translations
  AUTO_SAVE_THRESHOLD=200 # number of cache misses
  
  def __init__(self):
    self.translator = Translator()
    self.inflect = inflect.engine()
    self.filename=os.path.dirname(os.path.realpath(__file__))+'/translations.pickle'
    self.counter=0
    self.changed=False
    # read cache from pickle file
    try:
      f = open(self.filename+".new",'r')  
      self.cache = pickle.load(f)
      f.close()
    except:
      self.cache = {}
    print("Translator initial cache: " + str(len(self.cache)))

  def translate(self, label, lang='de'):
    try:
      val = self.get(label)
    except:
      try:
        translated = str(self.translator.translate(label, src=lang, dest='en').text)
        # remove non alphabetical characters
        translated = ''.join(filter(lambda e: e.isalpha() or e.isspace(), list(translated)))
        translatedWords = translated.split(' ')
        for i in range(len(translatedWords)):
          try:    singularNoun = self.inflect.singular_noun(translatedWords[i])
          except: continue
          if singularNoun!=False and singularNoun!=None:
            translatedWords[i] = singularNoun
        val = ' '.join(translatedWords)
      except: val = label
      self.set(label, val)
    return val
  
  def get(self, key):
    return self.cache[key]
  
  def set(self, key, value):
    self.cache[key] = value
    self.counter += 1
    self.changed = True
    if self.counter>LabelTranslator.AUTO_SAVE_THRESHOLD:
      self.counter=0
      self.save()
  
  def save(self):
    if not self.changed: return
    self.changed = False
    f = open(self.filename+".new",'wb')
    pickle.dump(self.cache,f)
    f.close()
    copyfile(self.filename+".new", self.filename)

class OWLResourceManager:
  """
  bla bla bla
  """
  def __init__(self, ontologyPrefix, ontologyURI):
    self.ontologyPrefix = ontologyPrefix
    self.ontologyURI = ontologyURI
    self.translator = LabelTranslator()
    self.owlClasses = {}
    self.owlIndividuals = {}
    self.outMode = 'OWL'
    self.imports = []
    self.namespaces = []
    self.register_namespace('rdf','http://www.w3.org/1999/02/22-rdf-syntax-ns#')
    self.register_namespace('rdfs','http://www.w3.org/2000/01/rdf-schema#')
    self.register_namespace('owl','http://www.w3.org/2002/07/owl#')
    self.register_namespace('xml','http://www.w3.org/XML/1998/namespace')
    self.register_namespace('xsd','http://www.w3.org/2001/XMLSchema#')
    self.register_namespace('knowrob','http://knowrob.org/kb/knowrob.owl#')
    self.register_namespace('computable','http://knowrob.org/kb/computable.owl#')
    self.register_namespace(ontologyPrefix,ontologyURI+'#')
  
  def register_namespace(self, prefix, URI):
    self.namespaces.append((prefix,URI))
  
  def add_import(self, ontologiFile):
    self.imports.append(ontologiFile)

  def set_output_mode(self, outMode):
    self.outMode = outMode
  
  def subclass_of(self, name1, name2):
    if name1 == name2: return True
    try:
      for parent in self.owlClasses[name1].types:
        if self.subclass_of(parent, name2):
          return True
    except: pass
    return False

  def cleanup_subclasses(self):
    for name in self.owlClasses:
      self.cleanup_types(self.owlClasses[name])
    #for name in self.owlIndividuals:
    #  self.cleanup_types(self.owlIndividuals[name])
  
  def cleanup_types(self, entity):
    cleaned = []
    unique = set(entity.types)
    for a in unique:
      skip = False
      for b in unique:
        if a==b: continue
        if self.subclass_of(b,a):
          skip=True
          break
      if skip == False: cleaned.append(a)
    entity.types = cleaned
  
  def get(self, label, lang='de'):
    clsName = self.get_name(label, lang)
    if clsName=='': return None
    try:
      x = self.owlClasses[clsName]
    except:
      x = OWLClass(clsName, self.ontologyPrefix)
      self.owlClasses[clsName] = x
    x.add_label(label)
    return x
  
  def get_instance(self, clsName, instanceName=''):
    try:
      return self.owlIndividuals[instanceName]
    except:
      x = OWLIndividual(instanceName, self.ontologyPrefix)
      x.has_type(clsName)
      self.owlIndividuals[x.name] = x
      return x
  
  def get_name(self, label, lang='de'):
    return self.owl_name(self.translator.translate(label, lang))
  
  def owl_name(self, label):
    return ''.join(label.title().split(' '))
  
  def dump(self, outFile, iri, filter):
    try:
      copyfile(outFile, outFile+".bak")
    except: pass
    f = open(outFile,'w')  
    if self.outMode=='OWL':
      self.dump_owl(f, iri, filter)
    elif self.outMode=='plain':
      self.dump_plain(f, iri, filter)
    else:
      print("ERROR: unknown output mode '"+self.outMode+"'.")
    f.close()
  
  def dump_plain__(self, f, filter, dict):
    for name in dict:
      entity = dict[name]
      if not filter(entity): entity.write_plain(file)
  def dump_owl__(self, f, filter, dict):
    for name in dict:
      entity = dict[name]
      if not filter(entity): entity.write_owl(f)
  
  def dump_plain(self, f, iri, filter):
    self.dump_plain__(f, filter, self.owlClasses)
    self.dump_plain__(f, filter, self.owlIndividuals)
  def dump_owl(self, f, iri, filter):
    f.write('<?xml version="1.0"?>\n')
    f.write('<!DOCTYPE rdf:RDF [\n')
    for (ns,uri) in self.namespaces:
      f.write('    <!ENTITY '+ns+' "'+uri+'" >\n')
    f.write(']>\n\n')
    # <rdf:RDF>
    f.write('<rdf:RDF xmlns="'+iri+'#"\n')
    f.write('      xml:base="'+iri+'"')
    for (ns,uri) in self.namespaces:
      f.write('\n      xmlns:'+ns+'="'+uri+'"')
    f.write('>\n')
    # <owl:Ontology>
    f.write('    <owl:Ontology rdf:about="'+iri+'">\n')
    f.write('        <owl:imports rdf:resource="package://knowrob_common/owl/knowrob.owl"/>\n')
    for importURI in self.imports:
      f.write('        <owl:imports rdf:resource="'+importURI+'"/>\n')
    f.write('    </owl:Ontology>\n')
    # </owl:Ontology>
    self.dump_owl__(f, filter, self.owlClasses)
    self.dump_owl__(f, filter, self.owlIndividuals)
    # </rdf:RDF>
    f.write('</rdf:RDF>')
  
  def entity_count(self, filter):
    return self.class_count(filter) + self.individual_count(filter)
  
  def class_count(self, filter):
    count=0
    for name in self.owlClasses:
      if not filter(self.owlClasses[name]): count+=1
    return count
  
  def individual_count(self, filter):
    count=0
    for name in self.owlIndividuals:
      if not filter(self.owlIndividuals[name]): count+=1
    return count

