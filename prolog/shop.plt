:- begin_tests(shop).

:- register_ros_package(knowrob_refills).

:- owl_parser:owl_parse('package://knowrob_refills/owl/dm-market-iai.owl').
:- rdf_db:rdf_register_ns(iai_shop, 'http://knowrob.org/kb/dm-market-iai.owl#', [keep(true)]).

:- use_module(library('shop')).

%%%%

test(shelf_layer_above1, forall(member(X, [
    ['DMShelfBottomLayer_1', 'DMShelfLayer_0'],
    ['DMShelfLayer_0', 'DMShelfLayer_1'],
    ['DMShelfLayer_1', 'DMShelfLayer_2'],
    ['DMShelfLayer_2', 'DMShelfLayer_3']
  ]))) :-
  [Below,Above] = X,
  atom_concat('http://knowrob.org/kb/dm-market-iai.owl#', Below, Below_iri),
  atom_concat('http://knowrob.org/kb/dm-market-iai.owl#', Above, Above_iri),
  shelf_layer_above(Below_iri, Above_iri).

test(shelf_layer_above2, [fail]) :-
  shelf_layer_above(iai_shop:'DMShelfLayer_2', iai_shop:'DMShelfLayer_1').

test(shelf_layer_above3, [fail]) :-
  shelf_layer_above(iai_shop:'DMShelfLayer_1', iai_shop:'DMShelfBottomLayer_1').

test(shelf_layer_above4, [fail]) :-
  shelf_layer_above(iai_shop:'DMShelfLayer_1', iai_shop:'DMShelfBottomLayer_1');
  shelf_layer_above(iai_shop:'DMShelfLayer_2', iai_shop:'DMShelfBottomLayer_1').

%%%%

test(shelf_layer_below1, forall(member(X, [
    ['DMShelfLayer_0', 'DMShelfBottomLayer_1'],
    ['DMShelfLayer_1', 'DMShelfLayer_0'],
    ['DMShelfLayer_2', 'DMShelfLayer_1'],
    ['DMShelfLayer_3', 'DMShelfLayer_2']
  ]))) :-
  [Above,Below] = X,
  atom_concat('http://knowrob.org/kb/dm-market-iai.owl#', Below, Below_iri),
  atom_concat('http://knowrob.org/kb/dm-market-iai.owl#', Above, Above_iri),
  shelf_layer_below(Above_iri, Below_iri).

%%%%

:- end_tests(shop).
