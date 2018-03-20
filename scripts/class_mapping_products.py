#!/usr/bin/python
# coding: utf8

# TODO: use this information
CLASS_NAME_MAPPING={
  # seems inflect thinks "Christma" is singular noun of "Christmas" :/
  'ChristmaItem': 'ChristmasItem',
  #'BabyClothe': 'BabyClothing',
  'Dres': 'Clothing',
  # these make only sense in plural as products, inflect does no know this
  'Sock': 'Socks',
  'KneeSock': 'KneeSocks',
  'Tight': 'Tights',
  'Sweet': 'Sweets',
  'Sunglass': 'Sunglasses',
  'ReadingGlass': 'ReadingGlasses',
  'Sneaker': 'Sneakers',
  'Snack': 'Snacks',
  'SunBlock': 'SunBlocker',
}

