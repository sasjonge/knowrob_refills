#!/usr/bin/python
# coding: utf8

import sys, os
import re
import getopt
from openpyxl import load_workbook

from class_mapping import OWLResourceManager, OWLClass, OWLIndividual
from label_mapping import LABEL_MAPPING_PREFIX, \
                          LABEL_MAPPING_SUFFIX, \
                          LABEL_MAPPING_WORD, \
                          LABEL_MAPPING_REPLACE

def clean_label(label):
  words = label.replace('-',' '). \
                replace('.',' '). \
                replace('(',' '). \
                replace(')',' '). \
                replace('+',' oder '). \
                replace('&',' oder '). \
                replace('/',' oder '). \
                split()
  # replace prefix (first word)
  try:    words[0]  = LABEL_MAPPING_PREFIX[words[0]]
  except: pass
  # replace suffix (last word)
  try:    words[-1] = LABEL_MAPPING_SUFFIX[words[-1]]
  except: pass
  # replace words
  for i in range(len(words)):
    try:    words[i] = LABEL_MAPPING_WORD[words[i]]
    except: pass
  # Avoid that words are repeated and
  # that the next is the suffix of the previous one
  last = ''
  withoutRepetition = []
  for x in words:
    if last.lower().endswith(x.lower()): continue
    last=x
    withoutRepetition.append(x)
  clean = ' '.join(withoutRepetition).strip()
  try:    return LABEL_MAPPING_REPLACE[clean]
  except: return clean

class ProductTableColumn:
  """
  bla bla bla
  """
  def __init__(self, name, labels,
               coltype='class',
               parents=[],
               role=None,
               param=None, paramType=None,
               cls=None):
    self.name         = name
    self.labels       = labels
    self.coltype   = coltype
    self.parents   = parents
    self.role      = role
    self.cls       = cls
    self.param     = param
    self.paramType = paramType
  
  def format_label(self, label):
    return clean_label(label)
  
  def match_label(self, label):
    for x in self.labels:
      if str(label)==str(x): return True
    return False
  
  def __repr__(self):
    return self.name

class ProductTable:
  """
  bla bla bla
  """
  COLUMN_META_DATA = [
    ProductTableColumn('assortment',
      labels=['Warensortiment'],
      cls='Product',
      role='type'),
    ProductTableColumn('area',
      labels=['Warenbereich'],
      cls='Product'),
    ProductTableColumn('subArea',
      labels=['Warenunterbereich'],
      cls='Product',
      parents=['area'],
      role='type'),
    ProductTableColumn('generalType',
      labels=['Warenklasse'],
      cls='Product'),
    ProductTableColumn('specificType',
      labels=['Warengruppe'],
      parents=['generalType'],
      role='type'),
    #ProductTableColumn('brand',
    #  labels=['Marke Dachmarke'],
    #  coltype='instance',
    #  cls='ProductBrand',
    #  role='brand'),
    #ProductTableColumn('subBrand',
    #  labels=['Marke Submarke'],
    #  coltype='instance',
    #  cls='ProductBrand',
    #  role='brand'),
    ProductTableColumn('width',
      labels=['Artikel Breite (DANf) cm'],
      coltype='parameter',
      param='widthOfProduct',
      paramType='&xsd;float'),
    ProductTableColumn('height',
      labels=['Artikel Hoehe (DANf) cm'],
      coltype='parameter',
      param='heightOfProduct',
      paramType='&xsd;float'),
    ProductTableColumn('depth',
      labels=['Artikel Laenge (DANf) cm'],
      coltype='parameter',
      param='depthOfProduct',
      paramType='&xsd;float'),
    ProductTableColumn('article',
      labels=['Artikel'],
      coltype='annotation',
      role='label'),
    ProductTableColumn('articleNumber',['DAN', 'EAN'],coltype='instance'),
  ]
  
  
  def __init__(self, resourceManager, xlsxFile, xlsxTable, lang='de'):
    self.resourceManager = resourceManager
    self.table = load_workbook(xlsxFile)[xlsxTable]
    self.lang  = lang
    self.rawLabels = set()
    # create ordered list of columns we have some meta information about
    self.header = []
    for row in self.table.iter_rows(min_row=1, max_row=1):
      self.header=map(lambda x: self.find_column(x.value), row)
  
  def article(self, articleNumber):
    articleClass = "ProductWithAN"+articleNumber
    try: return self.resourceManager.owlClasses[articleClass]
    except:
      article = OWLClass(articleClass, self.resourceManager.ontologyPrefix)
      article.has_type('Product')
      article.has_object_value("articleNumberOfProduct", self.article_number(articleNumber))
      self.resourceManager.owlClasses[articleClass] = article
      return article
  
  def article_number(self, articleNumber):
    x = self.resourceManager.get_instance('EuropeanArticleNumber',
                                          'EuropeanArticleNumber_'+articleNumber)
    x.has_data_value('ean',
                     'http://www.w3.org/2001/XMLSchema#string',
                     articleNumber)
    return x

  def unique_names(self, column):
    """ read all values of a column (may be many) """
    names=set()
    for row in self.table.iter_rows(min_row=2):
      names.add(self.read_value(row, column))
    out = list(names)
    out.sort()
    return out
  
  def read(self):
    """ read in the complete table, create ProductClass instances on the way """
    for row in self.table.iter_rows(min_row=2):
      # first get article class
      articleNumber = self.read_value(row, 'articleNumber')
      articleClass  = self.article(articleNumber)
      for i in range(len(self.header)):
        column = self.header[i]
        if column!=None and (column.role!=None or column.param!=None):
          self.read_cell(articleClass,row,column)

  def read_cell(self,article,row,column):
    rawLabel = self.read_value(row,column)
    label    = column.format_label(rawLabel)
    if column.coltype=='class':
      cellClass = self.resourceManager.get(label, lang=self.lang)
      if cellClass==None: return None
      self.read_cell_types(article,cellClass,row,column)
      self.read_cell_property(article,cellClass,column)
      self.rawLabels.add(rawLabel)
      # columns may have parent classes in the upper shopping ontology
      if column.cls!=None: cellClass.has_type(column.cls)
      return cellClass.name
    elif column.coltype=='instance' and column.cls!=None:
      cellInstance = self.resourceManager.get_instance(label, column.cls)
      if cellInstance==None: return None
      self.read_cell_types(article,cellInstance,row,column)
      self.read_cell_property(article,cellInstance,column)
    elif column.coltype=='parameter':
      self.read_cell_property(article,rawLabel,column)
    return label

  def read_cell_types(self,article,resource,row,column):
    for x in column.parents:
      resource.has_type(self.read_cell(article,row,self.get_column(x)))

  def read_cell_property(self,article,resource,column):
    if column.role=='type':
      article.has_type(str(resource))
    elif column.role!=None:
      article.has_object_value(column.role, str(resource))
    elif column.param!=None and column.paramType!=None:
      article.has_data_value(column.param, column.paramType, str(resource))

  def read_value(self, row, column):
    index=self.get_column_index(column)
    if index==None: return None
    val=row[index].value
    if not isinstance(val, basestring): val=unicode(val)
    return val.encode('utf-8')
  
  def get_column_index(self,col):
    i=0
    for x in self.header:
      if str(col)==str(x): return i
      i+=1
  
  def get_column(self,col):
    for x in self.header:
      if str(col)==str(x): return x
  
  def find_column(self,x):
    for col in ProductTable.COLUMN_META_DATA:
      if col.match_label(x): return col


def main(argv):
  resourceManager = OWLResourceManager(ontologyPrefix='shop', \
                                       ontologyURI='http://knowrob.org/kb/shop.owl')
  resourceManager.add_import('package://knowrob_refills/owl/shop.owl')
  resourceManager.add_import('package://knowrob_refills/owl/dm-market.owl')
  
  outDir = os.path.dirname(os.path.realpath(__file__))
  # handle commandline parameters
  try:
    opts, args = getopt.getopt(argv, "gl:t:o:m:",
      ["gen", "list=", "table=", "output-dir=", "output-mode="])
  except getopt.GetoptError:
    sys.exit(2)
  opt_gen = False
  opt_list = []
  opt_tables = []
  for opt, arg in opts:
    if opt in ("-g", "--gen"):
      opt_gen = True
    elif opt in ("-t", "--table"):
      opt_tables.append(arg)
    elif opt in ("-o", "--output-dir"):
      outDir = arg
    elif opt in ("-m", "--output-mode"):
      resourceManager.set_output_mode(arg)
    elif opt in ("-l", "--list"):
      opt_list.append(arg)
  # instantiate tables
  tables = []
  for table_arg in opt_tables:
    x = table_arg.split(":")
    tables.append(ProductTable(resourceManager, x[0], x[1]))
  # print out unique names
  for l in opt_list:
      print("LABEL_MAPPING_"+l.upper()+"={")
      for t in tables:
        column = t.get_column(l)
        for x in t.unique_names(l):
          x0=str(x)
          clean=clean_label(x0)
          if clean=="": continue
          label=column.format_label(x0)
          x1=str(resourceManager.get_name(label, suffix=column.suffix))
          print("  '"+clean+"': '"+x1+"',")
      print("}")
      resourceManager.translator.save()
  # print out unique names
  if opt_gen:
      print("Reading tables (this may take a while)...")
      for t in tables:
        t.read()
        resourceManager.translator.save()
      # dump into multiple ontologies
      isClass         = lambda e: isinstance(e, OWLClass)
      isIndividual    = lambda e: isinstance(e, OWLIndividual)
      isProductClass  = lambda e: e.name.startswith('ProductWithAN')
      voidFilter      = lambda e: False
      filter1 = lambda e: isIndividual(e) or isProductClass(e)
      filter2 = lambda e: isClass(e) and not isProductClass(e)
      print("Dumping ontology to " + outDir)
      resourceManager.dump(outDir+"/product-taxonomy.owl", filter1)
      resourceManager.dump(outDir+"/product-catalog.owl",  filter2)
      # some debug output at the end
      print("Printing ontology statistics:")
      print("    total classes:     " + str(resourceManager.class_count(voidFilter)))
      print("    total individuals: " + str(resourceManager.individual_count(voidFilter)))
      print("    product classes:   " + str(resourceManager.entity_count(lambda e: not isProductClass(e))))
      uniqueNames = set()
      for t in tables:
         for x in t.rawLabels: uniqueNames.add(x)
      print("    raw taxonomy:      " + str(len(uniqueNames)))
      print("    cleaned taxonomy:  " + str(resourceManager.entity_count(filter1)))
      print("    catalog resources: " + str(resourceManager.entity_count(filter2)))

if __name__ == "__main__":
  main(sys.argv[1:])

