/**  <module> shop

  Copyright (C) 2018 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  @author Daniel Beßler
  @license BSD
*/

% TODO:
%    - add conversation hook for ean/dan?
%    - use qudt and ensure unit is meters here.
%    - include information about how much space can be taken by objects in layers

:- module(shop,
    [ 
      owl_classify(r,r),
      rdfs_classify(r,r),
      shelf_layer_frame(r,r),
      shelf_layer_mounting(r),
      shelf_layer_standing(r),
      shelf_layer_above(r,r),
      shelf_layer_below(r,r),
      shelf_layer_part(r,r,r),
      shelf_layer_position(r,r,-),
      shelf_layer_separator(r,r),
      shelf_layer_mounting_bar(r,r),
      shelf_layer_label(r,r),
      shelf_layer_find_facing_at(r,r,r),
      shelf_facing(r,r),
      shelf_facing_product_type(r,r),
      compute_shelf_facing_product_type(r,r),
      shelf_facings_mark_dirty(r),
      shelf_facings_assert_adjacent_label(r),
      shelf_separator_insert(r,r),
      shelf_separator_insert(r,r,r),
      shelf_label_insert(r,r),
      shelf_label_insert(r,r,r),
      % computable properties
      comp_isSpaceRemainingInFacing(r,r),
      comp_facingPose(r,r),
      comp_facingWidth(r,r),
      comp_facingHeight(r,r),
      comp_facingDepth(r,r),
      comp_preferredLabelOfFacing(r,r),
      %%%%% rooming in
      belief_shelf_left_marker_at(r,r,r),
      belief_shelf_right_marker_at(r,r,r),
      belief_shelf_at(r,r,r),
      shelf_classify(r,+,+,+),
      shelf_with_marker(r,r),
      marker_tag_to_erp_id(r,r),
      shelf_with_erp_id(r,r),
      %shelf_estimate_pose/1,
      %%%%%
      belief_shelf_part_at/4,
      belief_shelf_part_at/5,
      belief_shelf_barcode_at(r,r,+,+,-),
      belief_shelf_barcode_at(r,r,+,+,-,+),
      product_spawn_front_to_back(r,r),
      product_spawn_front_to_back(r,r,r),
      %%%%%
      dan_gtin(r,r),
      product_to_gtin(r,r),
      create_article_type(r,r),
      create_article_type(r,r,r),
      create_article_number(r,r,r),
      create_article_number(r,r),
      article_number_of_dan(r,r),
      product_dimensions(r,r)
      ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('lang/computable')).
:- use_module(library('lang/query')).
:- use_module(library('lang/terms/holds')).
:- use_module(library('lang/terms/is_a')).
:- use_module(library('lang/terms/transitive')).
:- use_module(library('ros/tf/tf_plugin')).
:- use_module(library('db/tripledb')).
:- use_module(library('db/scope')).
:- use_module(library('model/OWL')).
:- use_module(library('reasoning/OWL/plowl/class')).
:- use_module(library('reasoning/OWL/plowl/property')).
:- use_module(library('model/SOMA/OBJ')).
:- use_module(library('model/DUL/Object')).
:- use_module(library('reasoning/spatial/distance')).
:- use_module(library('ros/marker/marker_plugin')).

% :-  rdf_meta  create_article_type(r,r,r),
%               belief_shelf_part_at(r,r,r,r),
%               belief_shelf_part_at(r,r,r,r,r).
    % shelf_layer_frame(r,r),
    % shelf_layer_above(r,r),
    % shelf_layer_below(r,r),
    % shelf_layer_mounting(r),
    % shelf_layer_standing(r),
    % shelf_layer_position(r,r,-),
    % shelf_layer_mounting_bar(r,r),
    % shelf_layer_label(r,r),
    % shelf_layer_separator(r,r),
    % shelf_facing_product_type(r,r),
    % shelf_layer_part(r,r,r),
    % belief_shelf_part_at(r,r,+,r),
    % belief_shelf_part_at(r,r,+,r,+),
    % belief_shelf_barcode_at(r,r,+,+,-),
    % belief_shelf_barcode_at(r,r,+,+,-,+),
    % product_type_dimension_assert(r,r,+),
    % product_spawn_front_to_back(r,r,r),
    % product_spawn_front_to_back(r,r),
    % shelf_classify(r,+,+,+),
    % shelf_with_marker(r,r),
    % rdfs_classify(r,r),
    % owl_classify(r,r),
    % shelf_facings_mark_dirty(r),
    % create_article_type(r,r,r),
    % create_article_type(r,r).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(shop, 'http://knowrob.org/kb/shop.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(dmshop, 'http://knowrob.org/kb/dm-market.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(dul,
		'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(soma,
		'http://www.ease-crc.org/ont/SOMA.owl#', [keep(true)]).

% TODO: should be somewhere else
% TODO: must work in both directions
xsd_float(Value, literal(
    type('http://www.w3.org/2001/XMLSchema#float', Atom))) :-
  atom(Value) -> Atom=Value ; atom_number(Atom,Value).
xsd_boolean(Atom, literal(
    type('http://www.w3.org/2001/XMLSchema#boolean', Atom))) :-
  atom(Atom).

prolog:message(shop(Entities,Msg)) -->
  %{ owl_readable(Entities,Entities_readable) },
  ['[shop.pl] ~w ~w'-[Msg,Entities]].
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ArticleNumber'

%create_article_number(ean(AN), ArticleNumber) :-
  %create_article_number_(literal(type('http://knowrob.org/kb/shop.owl#ean',AN)), ArticleNumber).
%create_article_number(dan(AN), ArticleNumber) :-
  %create_article_number_(literal(type('http://knowrob.org/kb/shop.owl#dan',AN)), ArticleNumber).
%create_article_number_(AN_XSD,ArticleNumber) :-
  %owl_has(ArticleNumber, shop:articleNumberString, AN_XSD), !.
%create_article_number_(AN_XSD, ArticleNumber) :-
  %strip_literal_type(AN_XSD, AN_atom),
  %atomic_list_concat([
    %'http://knowrob.org/kb/shop.owl#ArticleNumber_',
    %AN_atom], ArticleNumber),
  %rdf_assert(ArticleNumber, rdf:type, shop:'ArticleNumber'),
  %rdf_assert(ArticleNumber, rdf:type, owl:'NamedIndividual'),
  %rdf_assert(ArticleNumber, shop:articleNumberString, AN_XSD),
  %create_product_type(ArticleNumber, ProductType),
  %print_message(warning, shop([ProductType], 'Missing product type. Incomplete data?')).
  
atomize(A,A) :- atom(A), !.
atomize(T,A) :- term_to_atom(T,A).

create_article_number(GTIN,DAN,AN) :-
  atomize(GTIN,GTIN_atom),
  atomize(DAN,DAN_atom),
  tell(instance_of(AN, shop:'ArticleNumber')),
  tell(holds(AN, shop:gtin, GTIN_atom)),
  tell(holds(AN, shop:dan, DAN_atom)).

create_article_number(dan(DAN),AN) :-
  atomize(DAN,DAN_atom),
  tell(instance_of(AN, shop:'ArticleNumber')),
  tell(holds(AN, shop:dan, DAN_atom)).

create_article_number(gtin(GTIN),AN) :-
  atomize(GTIN,GTIN_atom),
  tell(instance_of(AN, shop:'ArticleNumber')),
  tell(holds(AN, shop:gtin, GTIN_atom)).

dan_gtin(DAN, GTIN):-
  triple(G, shop:gtin, GTIN), triple(G, shop:dan, DAN).

product_to_gtin(ProductType, GTIN):-
  subclass_of(ProductType, shop:'Product'), 
  subclass_of(ProductType, D), 
  has_description(D,value(shop:articleNumberOfProduct,ArticleNumber)),
  triple(ArticleNumber, shop:gtin, GTIN).

% TODO : Fix the shape of products in owl file before changing the below predicates
create_article_type(AN,[D,W,H],ProductType) :-
  create_article_type(AN,ProductType),
  % specify bounding box
  xsd_float(D,D_XSD),
  xsd_float(W,W_XSD),
  xsd_float(H,H_XSD),

  tell(has_description(W_R, 
    value(shop:widthOfProduct, W_XSD))),
  tell(subclass_of(ProductType,W_R)),

  tell(has_description(D_R, 
    value(shop:depthOfProduct, D_XSD))), 
  tell(subclass_of(ProductType,D_R)),

  tell(has_description(H_R, 
    value(shop:heightOfProduct, H_XSD))),
  tell(subclass_of(ProductType,H_R)). 

create_article_type(AN,ProductType) :-
  once((
    holds(AN,shop:gtin,NUM) ;
    holds(AN,shop:dan,NUM) )),
  atomic_list_concat([
    'http://knowrob.org/kb/shop.owl#UnknownProduct_',NUM], ProductType),
  
  tell(subclass_of(ProductType, shop:'Product')),
  
  tell(has_description(AN_R, 
    value(shop:articleNumberOfProduct, AN))),
  tell(subclass_of(ProductType,AN_R)).
  

article_number_of_dan(DAN,AN) :-
  holds(AN,shop:dan,DAN),
  instance_of(AN,shop:'ArticleNumber'),!.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLayer'

%% 
shelf_layer_mounting(ShelfLayer) :- instance_of(ShelfLayer, shop:'ShelfLayerMounting').
%% 
shelf_layer_standing(ShelfLayer) :- instance_of(ShelfLayer, shop:'ShelfLayerStanding').
shelf_layer_standing_bottom(ShelfLayer) :- instance_of(ShelfLayer, dmshop:'DMShelfBFloor'), !.

%% 
shelf_layer_frame(Layer, Frame) :-
  holds(Frame, soma:hasPhysicalComponent, Layer),
  instance_of(Frame, shop:'ShelfFrame'), !.
%%
shelf_facing(ShelfLayer, Facing) :-
  holds(Facing, shop:layerOfFacing, ShelfLayer).

%%
shelf_layer_separator(Layer,Part)    :- shelf_layer_part(Layer,shop:'ShelfSeparator',Part).
%%
shelf_layer_mounting_bar(Layer,Part) :- shelf_layer_part(Layer,shop:'ShelfMountingBar',Part).
%%
shelf_layer_label(Layer,Part)        :- shelf_layer_part(Layer,shop:'ShelfLabel',Part).
%%
shelf_layer_part(Layer, Type, Part) :-
  instance_of(Layer, shop:'ShelfLayer'),
  holds(Layer, soma:hasPhysicalComponent, Part),
  instance_of(Part, Type).

%% shelf_layer_position
%
% The position of some object on a shelf layer.
% Position is simply the x-value of the object's pose in the shelf layer's frame.
%
shelf_layer_position(Layer, Object, Position) :-
  holds(Layer, knowrob:frameName, LayerFrame),
  is_at(Object, [LayerFrame, [Position,_,_], _]), !.
  % object_pose(Object, [LayerFrame,_,[Position,_,_],_]).

shelf_facing_position(Facing,Pos) :-
  holds(Facing, shop:leftSeparator, Left), !,
  holds(Facing, shop:rightSeparator, Right), 
  holds(Facing, shop:layerOfFacing, Layer), !,
  shelf_layer_position(Layer, Left, Pos_Left),
  shelf_layer_position(Layer, Right, Pos_Right),
  Pos is 0.5*(Pos_Left+Pos_Right).
shelf_facing_position(Facing, Pos) :-
  holds(Facing, shop:mountingBarOfFacing, MountingBar), !,
  holds(Facing, shop:layerOfFacing, Layer),
  shelf_layer_position(Layer, MountingBar, Pos).

%% 
shelf_layer_above(ShelfLayer, AboveLayer) :-
  shelf_layer_sibling(ShelfLayer, min_positive_element, AboveLayer).
%% 
shelf_layer_below(ShelfLayer, BelowLayer) :-
  shelf_layer_sibling(ShelfLayer, max_negative_element, BelowLayer).

shelf_layer_sibling(ShelfLayer, Selector, SiblingLayer) :-
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  holds(ShelfFrame, knowrob:frameName, ShelfFrameName),
  is_at(ShelfLayer, [ShelfFrameName, [_ ,_, Pos], _]),
  %object_pose(ShelfLayer, [ShelfFrameName,_,[_,_,Pos],_]),
  findall((X,Diff), (
    triple(ShelfFrame, soma:hasPhysicalComponent, X),
    X \= ShelfLayer,
    is_at(X, [ShelfFrameName, [_,_,X_Pos], _]),
    %object_pose(X, [ShelfFrameName,_,[_,_,X_Pos],_]),
    Diff is X_Pos-Pos), Xs),
  call(Selector, Xs, (SiblingLayer,_)).

shelf_layer_neighbours(ShelfLayer, Needle, Selector, Positions) :-
  shelf_layer_position(ShelfLayer, Needle, NeedlePos),
  findall((X,D), (
    call(Selector, ShelfLayer, X),
    X \= Needle,
    shelf_layer_position(ShelfLayer, X, Pos_X),
    D is Pos_X - NeedlePos
  ), Positions).

shelf_facing_labels_update(ShelfLayer,Facing) :-
  forall((
    holds(Facing,shop:adjacentLabelOfFacing,Label);
    holds(Facing,shop:labelOfFacing,Label)),(
    shelf_label_insert(ShelfLayer,Label)
  )).

shelf_facings_mark_dirty(ShelfLayer) :-
  findall(X, (
    shelf_facing(ShelfLayer,X),
    shelf_facing_update(X)
  ), AllFacings),
  show_marker(AllFacings, AllFacings).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfSeparator'

%%
shelf_separator_insert(ShelfLayer,Separator) :-
  shelf_separator_insert(ShelfLayer,Separator,[update_facings]).
shelf_separator_insert(ShelfLayer,Separator,Options) :-
  shelf_layer_standing(ShelfLayer),
  % [X.pos - Separator.pos]
  shelf_layer_neighbours(ShelfLayer, Separator, shelf_layer_separator, Xs),
  ignore(max_negative_element(Xs, (LeftOf,_))),
  ignore(min_positive_element(Xs, (RightOf,_))),
  %
  (( holds(_,shop:rightSeparator,Separator) ;
     holds(_,shop:leftSeparator,Separator)) ->
     shelf_separator_update(ShelfLayer, Separator, [LeftOf,RightOf]);
     shelf_separator_add(ShelfLayer, Separator, [LeftOf,RightOf]) ),
  % update labelOfFacing relation
  ((holds(Y,shop:leftSeparator,Separator);
    holds(Y,shop:rightSeparator,Separator)) ->
    shelf_facing_labels_update(ShelfLayer,Y) ; true ),
    assert_object_shape_(Separator),
  ( member(update_facings,Options) ->
    shelf_facings_mark_dirty(ShelfLayer) ;
    true ).
   
shelf_separator_update(_, X, [LeftOf,RightOf]) :-
  % FIXME: could be that productInFacing changes, but highly unlikely.
  %        this is at the moment only updated when a facing is retracted.
  % LeftOf,RightOf unchanged if ...
  ( holds(Facing1,shop:rightSeparator,X) ->
  ( ground(LeftOf), holds(Facing1,shop:leftSeparator,LeftOf));    \+ ground(LeftOf)),
  ( holds(Facing2,shop:leftSeparator,X) ->
  ( ground(RightOf), holds(Facing2,shop:rightSeparator,RightOf)); \+ ground(RightOf)), !.
shelf_separator_update(Layer, X, [LeftOf,RightOf]) :-
  % LeftOf,RightOf changed otherwise
  shelf_separator_remove(X),
  shelf_separator_add(Layer, X, [LeftOf,RightOf]).

shelf_separator_add(Layer, X, [LeftOf,RightOf]) :-
  (ground(RightOf) -> shelf_facing_assert(Layer,[X,RightOf],_) ; true),
  (ground(LeftOf)  -> shelf_facing_assert(Layer,[LeftOf,X],_) ; true),
  ((ground([RightOf,LeftOf]),
    holds(Facing, shop:rightSeparator,RightOf),
    holds(Facing, shop:leftSeparator,LeftOf)) ->
    shelf_facing_retract(Facing) ; true ).

shelf_separator_remove(X) :-
  ( holds(LeftFacing,shop:rightSeparator,X) ->
    shelf_separator_remove_rightSeparator(LeftFacing,X); true ),
  ( holds(RightFacing,shop:leftSeparator,Y) ->
    shelf_separator_remove_leftSeparator(RightFacing,Y); true ).
shelf_separator_remove_rightSeparator(Facing,X) :-
  holds(Facing,shop:leftSeparator,Left),
  holds(NextFacing,shop:leftSeparator,X),
  tripledb_forget(NextFacing,shop:leftSeparator,X),
  tell(holds(NextFacing,shop:leftSeparator,Left)),
  shelf_facing_retract(Facing).
shelf_separator_remove_leftSeparator(Facing,X) :-
  holds(Facing,shop:rightSeparator,Right),
  holds(NextFacing,shop:rightSeparator,X),
  tripledb_forget(NextFacing,shop:rightSeparator,X),
  tell(holds(NextFacing,shop:rightSeparator,Right)),
  shelf_facing_retract(Facing).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfMountingBar'

shelf_mounting_bar_insert(ShelfLayer,MountingBar) :-
  shelf_mounting_bar_insert(ShelfLayer,MountingBar,[update_facings]).
shelf_mounting_bar_insert(ShelfLayer,MountingBar,Options) :-
  (( holds(Facing,shop:mountingBarOfFacing,MountingBar),
     shelf_mounting_bar_remove(MountingBar) );
     shelf_facing_assert(ShelfLayer,MountingBar,Facing)), !,
  % [X.pos - MountingBar.pos]
  shelf_layer_neighbours(ShelfLayer, MountingBar, shelf_layer_mounting_bar, Xs),
  ( min_positive_element(Xs, (X,_)) -> (
    % positive means that X is right of Separator
    tell(holds(Facing, shop:rightMountingBar, X)),
    holds(RightFacing, shop:mountingBarOfFacing, X),
    tripledb_forget(RightFacing, shop:leftMountingBar, _),
    tell(holds(RightFacing, shop:leftMountingBar, MountingBar))) ;
    true ),
  ( max_negative_element(Xs, (Y,_)) -> (
    % negative means that Y is left of Separator
    tell(holds(Facing, shop:leftMountingBar, Y)),
    holds(LeftFacing, shop:mountingBarOfFacing, Y),
    tripledb_forget(LeftFacing, shop:rightMountingBar, _),
    tell(holds(LeftFacing, shop:rightMountingBar, MountingBar))) ;
    true ),
    assert_object_shape_(MountingBar),
  % update labelOfFacing and productInFacing relations
  shelf_facing_labels_update(ShelfLayer,Facing),
  ( member(update_facings,Options) ->
    shelf_facings_mark_dirty(ShelfLayer) ;
    true ).

shelf_mounting_bar_remove(MountingBar) :-
  holds(Facing,shop:mountingBarOfFacing,MountingBar),
  (( holds(Facing, shop:rightMountingBar, Right),
     holds(Facing, shop:leftMountingBar, Left) ) -> (
     holds(RightFacing, shop:mountingBarOfFacing, Right),
     holds(LeftFacing, shop:mountingBarOfFacing, Left),
     tell(holds(RightFacing,shop:leftMountingBar,Left)),
     tell(holds(LeftFacing,shop:rightMountingBar,Right))
  );true),
  tripledb_forget(_,shop:leftMountingBar,MountingBar),
  tripledb_forget(_,shop:rightMountingBar,MountingBar).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLabel'

shelf_label_insert(ShelfLayer,Label) :-
  shelf_label_insert(ShelfLayer,Label,[update_facings]).
shelf_label_insert(ShelfLayer,Label,Options) :-
  tripledb_forget(_,shop:labelOfFacing,Label),
  % find the position of Label on the shelf
  shelf_layer_position(ShelfLayer, Label, LabelPos),
  has_type(Label, ShelfLabel),
  subclass_of(ShelfLabel, Restriction), has_description(Restriction, value(knowrob:'widthOfObject', LabelWidth)),
  LabelPosLeft  is LabelPos - 0.5*LabelWidth,
  LabelPosRight is LabelPos + 0.5*LabelWidth,
  % first find the facing under which the label was perceived, 
  % then assert labelOfFacing relation
  ( shelf_layer_find_facing_at(ShelfLayer,LabelPosLeft,LabeledFacingLeft) ->
    tell(holds(LabeledFacingLeft,shop:labelOfFacing,Label)) ;
    true ),
  ((shelf_layer_find_facing_at(ShelfLayer,LabelPosRight,LabeledFacingRight),
    LabeledFacingRight \= LabeledFacingLeft ) ->
    tell(holds(LabeledFacingRight,shop:labelOfFacing,Label)) ;
    true ),
  subclass_of(ShelfLabel, HeightRest), has_description(HeightRest, value(knowrob:'heightOfObject', LabelHeight)),
  subclass_of(ShelfLabel, DepthRest), has_description(DepthRest, value(knowrob:'depthOfObject', LabelDepth)),
  
  subclass_of(ShelfLabel, ColorRest), has_description(ColorRest, value(knowrob:'mainColorOfObject', Color)), 
  atomic_list_concat(ListColor,' ', Color), maplist(atom_number, ListColor, ObjectColor), 
  nth0(3, ObjectColor, _, RGBValue),
  
  assert_object_shape_(Label,LabelDepth, LabelWidth, LabelHeight, RGBValue),
  
  ( member(update_facings,Options) ->
    shelf_facings_mark_dirty(ShelfLayer) ;
    true ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ProductFacing'

shelf_facing_assert(ShelfLayer,[Left,Right],Facing) :-
  shelf_layer_standing(ShelfLayer), !,
  tell(instance_of(Facing, shop:'ProductFacingStanding')),
  rdf_split_url(_, ObjFrameName, Facing), 
  tell(holds(Facing, knowrob:frameName, ObjFrameName)),
  tell(holds(Facing, shop:leftSeparator, Left)),
  tell(holds(Facing, shop:rightSeparator, Right)),
  tell(holds(Facing, shop:layerOfFacing, ShelfLayer)).

shelf_facing_assert(ShelfLayer,MountingBar,Facing) :-
  shelf_layer_mounting(ShelfLayer), !,
  tell(instance_of(Facing, shop:'ProductFacingMounting')),
  tell(holds(Facing, shop:mountingBarOfFacing, MountingBar)),
  tell(holds(Facing, shop:layerOfFacing, ShelfLayer)).

shelf_facing_retract(Facing) :-
  triple(Facing, shop:layerOfFacing, ShelfLayer),
  tripledb_forget(Facing, shop:layerOfFacing, _),
  % remove products from facing
  forall(triple(Product,shop:productInFacing,Facing),  
         shelf_layer_insert_product(ShelfLayer,Product)),
  % TODO: don't forget about this facing, also marker remve msg should be generated
  tripledb_forget(Facing, _, _). 

shelf_layer_insert_product(ShelfLayer,Product) :-
  shelf_layer_position(ShelfLayer,Product,Pos_Product),
  ( shelf_layer_find_facing_at(ShelfLayer,Pos_Product,Facing) ->
    tell(holds(Facing, shop:productInFacing, Product)) ;
    true ).

shelf_layer_find_facing_at(ShelfLayer,Pos,Facing) :-
  %% Remove Later, Facing values are repeated
  findall(F, holds(F, shop:layerOfFacing, ShelfLayer), AllFacings),
  list_to_set(AllFacings, FacingsInLayer),
  findall(F, 
    (member(F, FacingsInLayer),
    shelf_facing_position(F,FacingPos),
    shelf_facing_width(F, FacingWidth),
    Width is FacingWidth+0.02,
    FacingPos-0.5*Width =< Pos, Pos =< FacingPos+0.5*Width), 
    Facings),
  member(Facing, Facings).

facing_space_remaining_in_front(Facing,Obj) :-
  is_at(Obj, [_,[_,Obj_pos,_], _]), !,
  % object_pose(Obj, [_,_,[_,Obj_pos,_],_]),
  instance_of(Obj, Product),
  product_dimensions(Product,[Obj_depth,_,_]),
  object_dimensions(Facing,Facing_depth,_,_),
  once(Obj_pos > Obj_depth + 0.06 - 0.5*Facing_depth).

facing_space_remaining_behind(Facing,Obj) :-
  is_at(Obj, [_, [_,Obj_pos, _],_]), !,
  % object_pose(Obj, [_,_,[_,Obj_pos,_],_]),
  instance_of(Obj, Product),
  product_dimensions(Product,[Obj_depth,_,_]),
  object_dimensions(Facing,Facing_depth,_,_),
  once(Obj_pos < Facing_depth*0.5 - Obj_depth - 0.06).

shelf_facings_assert_adjacent_label(ShelfLayer) :-
  findall(Facing, 
    ( holds(Facing, shop:layerOfFacing, ShelfLayer),
      \+ holds(Facing, shop:labelOfFacing, Label)),
  AllFacings),
  list_to_set(AllFacings, Facings),
  forall( member(FacingWithoutLabel, Facings),
  ( % Get neighbours
    get_adjacent_label_of_facing(FacingWithoutLabel, ShelfLayer, Label),
    tell(triple(FacingWithoutLabel, shop:adjacentLabelOfFacing, Label)))
  ).

%%% TODO : if there are no labels for the neighbouring facings, get the labels from the layer, find the closest and assign it to facing
get_adjacent_label_of_facing(Facing, ShelfLayer, AdjacentLabel) :-
  triple(Facing, shop:leftSeparator, Left), 
  triple(Facing, shop:rightSeparator, Right),
  shelf_facing_position(Facing, FacingPosition),
  (
  (triple(FacingLeft, shop:rightSeparator, Left), 
  triple(FacingRight, shop:leftSeparator, Right),
  
  triple(FacingLeft, shop:labelOfFacing, LabelLeft), 
  triple(FacingRight, shop:labelOfFacing, LabelRight), 
  
  shelf_layer_position(ShelfLayer, LabelLeft, LeftFacingLabelPos),
  shelf_layer_position(ShelfLayer, LabelRight, RightFacingLabelPos),
    
  (abs(FacingPosition- LeftFacingLabelPos) > abs(FacingPosition- RightFacingLabelPos) -> 
  (AdjacentLabel = LabelRight); 
  AdjacentLabel = LabelLeft ));
  
  triple(FacingLeft, shop:rightSeparator, Left), triple(FacingLeft, shop:labelOfFacing, AdjacentLabel) ; 
  
  triple(FacingRight, shop:leftSeparator, Right), triple(FacingRight, shop:labelOfFacing, AdjacentLabel)
  );
  true.

%% shelf_facing_product_type
%
% dont judge me :(
shelf_facing_product_type(Facing, ProductType) :-
  triple(Facing, shop:productLabelOfFacing, ProductType), !.
shelf_facing_product_type(Facing, ProductType) :-
  compute_shelf_facing_product_type(Facing, ProductType), !.
shelf_facing_product_type(Facing, _) :-
  print_message(warning, shop([Facing], 'Facing has no associated product type.')),
  fail.

compute_shelf_facing_product_type(Facing, ProductType) :-
  comp_preferredLabelOfFacing(Facing,Label),
  holds(Label,shop:articleNumberOfLabel,ArticleNumber),
  subclass_of(ProductType, shop:'Product'),
  subclass_of(ProductType, R),
  has_description(R,value(shop:articleNumberOfProduct,ArticleNumber)), !,
  tell(triple(Facing, shop:productLabelOfFacing, ProductType)).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% COMPUTABLES

%% comp_preferredLabelOfFacing
%
comp_preferredLabelOfFacing(Facing,Label) :-
  ground(Facing),
  holds(Facing,shop:labelOfFacing,Label).
  % preferred if label is one of the labelOfFacing
  %% findall(L,holds(Facing,shop:labelOfFacing,L),[X|Xs]), !,
  %% member(Label,[X|Xs]).
comp_preferredLabelOfFacing(Facing,Label) :-
  ground(Facing), !,
  holds(Facing,shop:adjacentLabelOfFacing,Label).
comp_preferredLabelOfFacing(Facing,Label) :-
  holds(Facing,shop:labelOfFacing,Label).
comp_preferredLabelOfFacing(Facing,Label) :-
  % preferred if label is adjacent to facing without labelOfFacing
  holds(Facing,shop:adjacentLabelOfFacing,Label),
  \+ holds(Facing,shop:labelOfFacing,_).

%% comp_isSpaceRemainingInFacing
%
comp_isSpaceRemainingInFacing(Facing,Val_XSD) :-
  shelf_facing_products(Facing, ProductsFrontToBack),
  ((ProductsFrontToBack=[] ; (
    reverse(ProductsFrontToBack, ProductsBackToFront),
    ProductsFrontToBack=[(_,First)|_],
    ProductsBackToFront=[(_,Last)|_], (
    once(facing_space_remaining_in_front(Facing,First));
    once(facing_space_remaining_behind(Facing,Last)))
  )) ->
  xsd_boolean('true',Val_XSD);
  xsd_boolean('false',Val_XSD)),!.

shelf_facing_update(Facing) :-
  % update geometry
  comp_facingDepth(Facing,D),
  comp_facingWidth(Facing,W),
  comp_facingHeight(Facing,H),
  
  % tell(holds(Facing, knowrob:'depthOfObject', D)),
  % tell(holds(Facing, knowrob:'widthOfObject', W)),
  % tell(holds(Facing, knowrob:'heightOfObject', H)),
  tell(has_type(FacingShape, soma:'Shape')),
  tell(triple(Facing, soma:hasShape, FacingShape)),
  tell(object_dimensions(Facing, D,W,H)),
  % update pose
  comp_facingPose(Facing, Pose),
  tell(is_at(Facing,Pose)),
  % preferred label
  tripledb_forget(Facing,shop:preferredLabelOfFacing,Label),
  ( comp_preferredLabelOfFacing(Facing,Label) ->
    tell(holds(Facing,shop:preferredLabelOfFacing,Label)) ;
    true
  ),
  % update color
  comp_mainColorOfFacing(Facing,Color),
  nth0(3, Color, _, RGB),
  once(assert_object_shape_(Facing, D,W, H, RGB)),!.
  % tell(object_color_rgb(Facing,Color)).

%% comp_facingPose
%
comp_facingPose(Facing, [FloorFrame,[Pos_X,Pos_Y,Pos_Z],[0.0,0.0,0.0,1.0]]) :-
  holds(Facing, shop:leftSeparator, _), !,
  holds(Facing, shop:layerOfFacing, Layer),
  holds(Facing, knowrob:frameName, FacingFrame),
  holds(Layer, knowrob:frameName, FloorFrame),
  object_dimensions(Facing, _, _, Facing_H),
  %holds(Facing, knowrob:'heightOfObject', Facing_H),
  shelf_facing_position(Facing, Pos_X),
  Pos_Y is -0.02,               % 0.06 to leave some room at the front and back of the facing
  % FIXME: should be done by offset, also redundant with spawn predicate
  ( shelf_layer_standing_bottom(Layer) ->
    Pos_Z is 0.5*Facing_H + 0.025 ;
    Pos_Z is 0.5*Facing_H + 0.08 ).
comp_facingPose(Facing, [FloorFrame, [Pos_X,Pos_Y,Pos_Z],[0.0,0.0,0.0,1.0]]) :-
  holds(Facing, shop:mountingBarOfFacing, _), !,
  holds(Facing, shop:layerOfFacing, Layer),
  holds(Facing, knowrob:frameName, FacingFrame),
  holds(Layer, knowrob:frameName, FloorFrame),
  comp_facingHeight(Facing, Facing_H),
  shelf_facing_position(Facing,Pos_X),
  Pos_Y is -0.03,           
  Pos_Z is -0.5*Facing_H.
  
%% comp_facingWidth
%
comp_facingWidth(Facing, Value) :-
  shelf_facing_width(Facing,Value).

shelf_facing_width(Facing, Value) :-
  atom(Facing),
  holds(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  holds(Facing, shop:leftSeparator, Left), !,
  holds(Facing, shop:rightSeparator, Right), !,
  shelf_layer_position(ShelfLayer, Left, Pos_Left),
  shelf_layer_position(ShelfLayer, Right, Pos_Right),
  Value is abs(Pos_Right - Pos_Left) - 0.02. % leave 2cm to each side
shelf_facing_width(Facing, Value) :-
  holds(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_mounting(ShelfLayer), !,
  holds(Facing, shop:mountingBarOfFacing, MountingBar),
  shelf_layer_position(ShelfLayer, MountingBar, MountingBarPos),
  object_dimensions(ShelfLayer, _, LayerWidth, _),
  ( holds(Facing, shop:leftMountingBar, Left) ->
    shelf_layer_position(ShelfLayer, Left, LeftPos) ;
    LeftPos is -0.5*LayerWidth
  ),
  ( holds(Facing, shop:rightMountingBar, Right) ->
    shelf_layer_position(ShelfLayer, Right, RightPos) ;
    RightPos is 0.5*LayerWidth
  ),
  Value is min(MountingBarPos - LeftPos,
               RightPos - MountingBarPos). % leave 1cm to each side

%% comp_facingHeight
%
comp_facingHeight(Facing, Value) :-
  atom(Facing),
  holds(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  holds(ShelfFrame, knowrob:frameName, ShelfFrameName),
  is_at(ShelfLayer, [ShelfFrameName, [_,_, X_Pos], _]),
  % object_pose(ShelfLayer, [ShelfFrameName,_,[_,_,X_Pos],_]),
  % compute distance to layer above
  ( shelf_layer_above(ShelfLayer, LayerAbove) -> (
    is_at(LayerAbove, [ShelfFrameName, [_, _, Y_Pos], _]),
    % object_pose(LayerAbove, [ShelfFrameName,_,[_,_,Y_Pos],_]),
    Distance is abs(X_Pos-Y_Pos)) ; (
    % no layer above
    object_dimensions(ShelfFrame, _, _, Frame_H),
    Distance is 0.5*Frame_H - X_Pos
  )),
  % compute available space for this facing
  ( shelf_layer_standing(LayerAbove) -> % FIXME could be unbound
    % above is also standing layer, whole space can be taken TODO minus layer height
    Value is Distance - 0.1;
    % above is mounting layer, space must be shared. HACK For now assume equal space sharing
    Value is 0.5*Distance - 0.1
  ), !.
comp_facingHeight(Facing, Value) :-
  atom(Facing),
  holds(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_mounting(ShelfLayer), !,
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  holds(ShelfFrame,knowrob:frameName, ShelfFrameName),
  is_at(ShelfLayer, [ShelfFrameName, [_,_,X_Pos], _]),
  % object_pose(ShelfLayer, [ShelfFrameName,_,[_,_,X_Pos],_]),
  % compute distance to layer above
  ( shelf_layer_below(ShelfLayer, LayerBelow) -> (
    is_at(LayerBelow, [ShelfFrameName,[_,_,Y_Pos],_]),
    % object_pose(LayerBelow, [ShelfFrameName,_,[_,_,Y_Pos],_]),
    Distance is abs(X_Pos-Y_Pos)) ; (
    % no layer below
    object_dimensions(ShelfFrame, _, _, Frame_H),
    Distance is 0.5*Frame_H + X_Pos
  )),
  % compute available space for this facing
  ( shelf_layer_mounting(LayerBelow) ->  % FIXME could be unbound
    % below is also mounting layer, whole space can be taken TODO minus layer height
    Value is Distance - 0.1;
    % below is standing layer, space must be shared. HACK For now assume equal space sharing
    Value is 0.5*Distance - 0.1
  ).

%% comp_facingDepth
%
comp_facingDepth(Facing, Value) :- comp_facingDepth(Facing, shelf_layer_standing, -0.06, Value).
comp_facingDepth(Facing, Value) :- comp_facingDepth(Facing, shelf_layer_mounting, 0.0, Value).
comp_facingDepth(Facing, Selector, Offset, Value) :-
  atom(Facing),
  holds(Facing, shop:layerOfFacing, ShelfLayer),
  call(Selector, ShelfLayer), !,
  object_dimensions(ShelfLayer, Value0, _, _),
  Value is Value0 + Offset.

comp_mainColorOfFacing(Facing, Color) :-
  holds(Facing, shop:layerOfFacing, _), !,
  ((has_type(Facing, shop:'UnlabeledProductFacing'),Color=[1.0, 0.35, 0.0, 0.5]);
  % FIXME: MisplacedProductFacing seems slow, comp_MisplacedProductFacing is a bit faster
   (comp_MisplacedProductFacing(Facing),Color=[1.0, 0.0, 0.0, 0.5]);
   (has_type(Facing, shop:'EmptyProductFacing'),Color=[1.0, 1.0, 0.0, 0.5]);
   (has_type(Facing, shop:'FullProductFacing'),Color=[0.0, 0.25, 0.0, 0.5]);
   Color=[0.0, 1.0, 0.0, 0.5]), !.

comp_MisplacedProductFacing(Facing) :-
  holds(Facing,shop:productInFacing,Item),
  instance_of(Item,Product),
  subclass_of(Product, R), is_restriction(R, value(shop:articleNumberOfProduct,AN)), 
  %holds(Product,shop:articleNumberOfProduct,AN),
  forall( holds(Label,shop:articleNumberOfLabel,AN),
          \+ comp_preferredLabelOfFacing(Facing,Label) ),!.
comp_MisplacedProductFacing(Facing) :-
  holds(Facing,shop:productInFacing,Item),
  instance_of(Item,Product),
  \+ subclass_of(Product, R), is_restriction(R, value(shop:articleNumberOfProduct,_)),!.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% "rooming in" -- perceive shelf barcodes and classify shelves
% TODO: make marker pose relative
% TODO: shelf pose computation

shelf_marker_offset(0.04).

shelf_type(LeftMarker,RightMarker,ShelfType) :-
  ValidWidths=[
    [0.65,'http://knowrob.org/kb/dm-market.owl#DMShelfW60'],
    [0.73,'http://knowrob.org/kb/dm-market.owl#DMShelfW70'],
    [1.0,'http://knowrob.org/kb/dm-market.owl#DMShelfW100'],
    [1.2,'http://knowrob.org/kb/dm-market.owl#DMShelfW120']
  ],
  % estimate width from markers
  object_distance(LeftMarker,RightMarker,Distance),
  shelf_marker_offset(Offset),
  WidthEstimate is Distance + Offset,
  % find closest match
  findall([D,Type], (
    member([Width,Type],ValidWidths),
    D is abs(WidthEstimate - Width)),
    ShelfTypes),
  sort(ShelfTypes, [[W,ShelfType]|_]),
  tell(holds(ShelfType, knowrob:widthOfObject, W)).

shelf_with_marker(Shelf,Marker) :- (
  holds(Shelf,dmshop:leftMarker,Marker);
  holds(Shelf,dmshop:rightMarker,Marker)),!.
shelf_with_marker(Shelf,Id) :-
  atom(Id),
  holds(Marker,dmshop:markerId,Id),
  shelf_with_marker(Shelf,Marker),!.

shelf_marker(Id,Marker):-
  atom(Id), holds(Marker,dmshop:markerId,Id),!.
shelf_marker(Marker,Marker):-
  atom(Marker), holds(Marker,dmshop:markerId,_),!.

marker_tag_to_erp_id(Tag, ID):-
  ground(Tag), 
  split_string(Tag, '_','',L), 
  member(NasString, L), 
  atom_number(NasString, N), 
  ((N mod 2 =:= 0) -> (Y is N-1);(Y = N)),
  ID is (Y+1)/2.
marker_tag_to_erp_id(Tag, ID):-
  ground(ID), 
  ID2 is ID*2-1,
  atom_string(ID2, IDasString), 
  string_concat('tag_', IDasString, Tag_str), 
  atom_string(Tag, Tag_str).

shelf_with_erp_id(Shelf, ID):-
  marker_tag_to_erp_id(Tag, ID), 
  shelf_with_marker(Shelf, Tag), !.
shelf_with_erp_id(Shelf, ID):-
  shelf_with_marker(Shelf, Marker), 
  holds(Marker, dmshop:markerId, Tag), 
  marker_tag_to_erp_id(Tag, ID), !.

%%
%
belief_shelf_marker_at(MarkerType,MarkerId,PoseData,Marker):-
  belief_new_object(MarkerType, Marker),
  tell(holds(Marker, dmshop:markerId, MarkerId)),
  tell(is_at(Marker,PoseData)). % Pose here is [RefFrame, Translation, Quat]
%%
%

belief_shelf_left_marker_at(Pose,MarkerId,Marker):-
  rdf_equal(MarkerType,dmshop:'DMShelfMarkerLeft'),
  belief_shelf_marker_at(MarkerType,MarkerId,Pose,Marker).
%%
%
belief_shelf_right_marker_at(Pose,MarkerId,Marker):-
  rdf_equal(MarkerType,dmshop:'DMShelfMarkerRight'),
  belief_shelf_marker_at(MarkerType,MarkerId,Pose,Marker).

%%
%
belief_shelf_at(LeftMarkerId,RightMarkerId,Shelf) :-
  shelf_marker(LeftMarkerId,LeftMarker),
  shelf_marker(RightMarkerId,RightMarker),
  once((
    % asserted before
    shelf_with_marker(Shelf,LeftMarker);
    shelf_with_marker(Shelf,RightMarker);
    % new shelf
    belief_new_shelf_at(LeftMarkerId,RightMarkerId,Shelf))).

belief_new_shelf_at(LeftMarkerId,RightMarkerId,Shelf) :-
  % infer shelf type (e.g. 'DMShelfFrameW100')
  shelf_marker(LeftMarkerId,LeftMarker),
  shelf_marker(RightMarkerId,RightMarker),
  shelf_type(LeftMarker,RightMarker,ShelfType),
  % assert to belief state and link marker to shelf
  belief_new_object(ShelfType,Shelf),
  % temporary assert height/depth
  % FIXME: what is going on here? why assert? rather use object_* predicates
  rdfs_classify(Shelf,ShelfType),
  tell(holds(Shelf, knowrob:depthOfObject, '0.02')),
  tell(holds(Shelf, knowrob:heightOfObject, '0.11')),
  tell(holds(Shelf, knowrob:mainColorOfObject, '0.0 1.0 0.5 0.6')),
  tell(holds(Shelf, dmshop:leftMarker, LeftMarker)),
  tell(holds(Shelf, dmshop:rightMarker, RightMarker)).


shelf_find_type(Shelf, Type) :-
  subclass_of(Type,dmshop:'DMShelfFrame'),
  forall(
      (has_type(Shelf, X), 
      subclass_of(X, dmshop:'DMShelfFrame')),
      transitive(subclass_of(Type, X))
    ).

%%
%
shelf_classify(Shelf,Height,NumTiles,Payload) :-
  % retract temporary height/depth
  tripledb_forget(Shelf, knowrob:depthOfObject, _),
  tripledb_forget(Shelf, knowrob:heightOfObject, _),
  tripledb_forget(Shelf, knowrob:mainColorOfObject, _),
  % assert various types based on input.
  % this fails in case input is not valid.
  shelf_classify_height(Shelf,Height),
  shelf_classify_num_tiles(Shelf,NumTiles),
  shelf_classify_payload(Shelf,Payload),
  % use closed world semantics to infer all the shelf frame
  % types currently implied for `Shelf`
  ( shelf_find_type(Shelf,ShelfType) -> (
    print_message(info, shop([Shelf,ShelfType], 'Is classified as.')),
    rdfs_classify(Shelf,ShelfType));(
    findall(X,(
      triple(Shelf,rdf:type,X),
      transitive(subclass_of(X,dmshop:'DMShelfFrame'))),Xs),
    print_message(warning, shop([Shelf,Xs], 'Failed to classify. Type not defined in ontology?'))
  )),

  %%%% Assert Shelf dimensions
    
  subclass_of(Type1, dmshop:'DMShelfFrame'), transitive(subclass_of(ShelfType, Type1)),
  subclass_of(Type1, HeightDesc), has_description(HeightDesc, value(knowrob:heightOfObject, Height)),
  
  subclass_of(Type2, dmshop:'DMShelfFrame'), transitive(subclass_of(ShelfType, Type2)),
  subclass_of(Type2, WidthDesc), has_description(WidthDesc, value(knowrob:widthOfObject, Width)), 

  subclass_of(Type3, dmshop:'DMShelfFrame'), transitive(subclass_of(ShelfType, Type3)),
  subclass_of(Type3, DepthDesc), has_description(DepthDesc, value(knowrob:depthOfObject, Depth)), 

  tell(has_type(Shape, soma:'Shape')),
  tell(holds(Shelf,soma:hasShape,Shape)),
  tell(object_dimensions(Shelf, Depth, Width, Height)),
  holds(Shape,dul:hasRegion,ShapeRegion),
 
  %%%% Assert object shape to get the object markers
  
  subclass_of(ShelfType, MeshDesc), has_description(MeshDesc, value(knowrob:pathToCadModel, FilePath)), 
  tell(holds(ShapeRegion,soma:hasFilePath,FilePath)),
  Pos = [0,0,0], Rot = [0,0,0,1],
  tell(is_individual(Origin)),
  tell(triple(ShapeRegion,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin)),
	tell(triple(Origin, soma:hasPositionVector, term(Pos))),
	tell(triple(Origin, soma:hasOrientationVector, term(Rot))),
 
  %%%% Assert perception feature
  assert_perception_feature_(Shelf), !,
  marker_plugin:republish.

%%
% Classify shelf based on its height.
shelf_classify_height(Shelf,1.6):-
  rdfs_classify(Shelf, dmshop:'DMShelfH160'),!.
shelf_classify_height(Shelf,1.8):-
  rdfs_classify(Shelf, dmshop:'DMShelfH180'),!.
shelf_classify_height(Shelf,2.0):-
  rdfs_classify(Shelf, dmshop:'DMShelfH200'),!.
shelf_classify_height(_Shelf,H):-
  print_message(warning, shop([H], 'Is not a valid shelf height (one of [1.6,1.8,2.0]).')),
  fail.

%%
% Classify shelf based on number of tiles in bottom floor.
shelf_classify_num_tiles(Shelf,4):-
  rdfs_classify(Shelf, dmshop:'DMShelfT4'),!.
shelf_classify_num_tiles(Shelf,5):-
  rdfs_classify(Shelf, dmshop:'DMShelfT5'),!.
shelf_classify_num_tiles(Shelf,6):-
  rdfs_classify(Shelf, dmshop:'DMShelfT6'),!.
shelf_classify_num_tiles(Shelf,7):-
  rdfs_classify(Shelf, dmshop:'DMShelfT7'),!.
shelf_classify_num_tiles(_Shelf,Count):-
  print_message(warning, shop([Count], 'Is not a valid number of shelf tiles (one of [5,6,7]).')),
  fail.

%%
% Classify shelf based on payload.
shelf_classify_payload(Shelf,heavy) :-
  rdfs_classify(Shelf, dmshop:'DMShelfH'),!.
shelf_classify_payload(Shelf,light) :-
  rdfs_classify(Shelf, dmshop:'DMShelfL'),!.
shelf_classify_payload(_Shelf,Mode):-
  print_message(warning, shop([Mode], 'Is not a valid payload mode (one of [heavy,light]).')),
  fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% belief_state part of shelves

%%
%
% This predicate exists to establish some relations
% between labels and facings, and to create facings
% between separators and mounting bars.
%
belief_shelf_part_at(Frame, Type, Pos, Obj) :-
  belief_shelf_part_at(Frame, Type, Pos, Obj, [insert,update_facings]).

belief_shelf_part_at(Frame, Type, Pos, Obj, _Options) :-
  transitive(subclass_of(Type, shop:'ShelfLayer')), !,
  pos_term(y,Pos,PosTerm),
  perceived_part_at_axis__(Frame, Type, PosTerm, Obj), 
  rdf_split_url(_,ObjFrame,Obj),

  subclass_of(Type, ShelfType1), transitive(subclass_of(ShelfType1, shop:'ShelfLayer')),
  subclass_of(ShelfType1, R1), has_description(R1, value(knowrob:'heightOfObject', H)), !,

  subclass_of(Type, ShelfType2), transitive(subclass_of(ShelfType2, shop:'ShelfLayer')),
  subclass_of(ShelfType2, R2), has_description(R2, value(knowrob:'depthOfObject', D)), !,

  subclass_of(Type, ShelfType3), transitive(subclass_of(ShelfType3, shop:'ShelfLayer')),
  subclass_of(ShelfType3, R3), has_description(R3, value(knowrob:'widthOfObject', W)), !,

  tell(holds(Obj, knowrob:frameName, ObjFrame)),
  
  % tell(holds(Obj, knowrob:depthOfObject, D)),
  % tell(holds(Obj, knowrob:heightOfObject, H)),
  % tell(holds(Obj, knowrob:widthOfObject, W)),
  tell(has_type(ShelfShape,soma:'Shape')),
  tell(triple(Obj,soma:hasShape,ShelfShape)),
  tell(object_dimensions(Obj, D, W, H)),
  once(assert_object_shape_(Obj)),
  once(assert_perception_feature_(Obj)), !,
  % adding a new shelf floor has influence on facing size of
  % shelf floor siblings
  ( shelf_layer_below(Obj,Below) ->
    shelf_facings_mark_dirty(Below) ; true ),
  ( shelf_layer_above(Obj,Above) ->
    shelf_facings_mark_dirty(Above) ; true ),
  marker_plugin:republish.

belief_shelf_part_at(Layer, Type, Pos, Obj, Options) :-
  transitive(subclass_of(Type, shop:'ShelfSeparator')), !,
  pos_term(x,Pos,PosTerm),
  perceived_part_at_axis__(Layer, Type, PosTerm, Obj),
  ( member(insert,Options) ->
    shelf_separator_insert(Layer,Obj,Options) ;
    true ),
  belief_new_object(Type, Obj),
  assert_perception_feature_(Obj), !.
 

belief_shelf_part_at(Layer, Type, Pos, Obj, Options) :-
  transitive(subclass_of(Type, shop:'ShelfMountingBar')), !,
  pos_term(x,Pos,PosTerm),
  perceived_part_at_axis__(Layer, Type, PosTerm, Obj),
  ( member(insert,Options) ->
    shelf_mounting_bar_insert(Layer,Obj,Options) ;
    true ),
  belief_new_object(Type, Obj),
  assert_perception_feature_(Obj), !.


belief_shelf_part_at(Layer, Type, Pos, Obj, Options) :- 
  transitive(subclass_of(Type, shop:'ShelfLabel')), !,
  pos_term(x,Pos,PosTerm),
  perceived_part_at_axis__(Layer, Type, PosTerm, Obj),
  ( member(insert,Options) ->
    shelf_label_insert(Layer,Obj,Options) ;
    true ).

%%
%%

perceived_pos__([DX,DY,DZ], pos(x,P), [X,DY,DZ]) :- X is DX+P, !.
perceived_pos__([DX,DY,DZ], pos(z,P), [DX,Y,DZ]) :- Y is DY+P, !.
perceived_pos__([DX,DY,DZ], pos(y,P), [DX,DY,Z]) :- Z is DZ+P, !.

denormalize_part_pos(Obj, x, In, Out) :-
  object_dimensions(Obj,_,V,_), Out is V*In.
denormalize_part_pos(Obj, y, In, Out) :-
  object_dimensions(Obj,_,_,V), Out is V*In.
denormalize_part_pos(Obj, z, In, Out) :-
  object_dimensions(Obj,V,_,_), Out is V*In.

center_part_pos(Obj, x, In, Out) :-
  object_dimensions(Obj,_,V,_),
  Out is In - 0.5*V.
center_part_pos(Obj, y, In, Out) :-
  object_dimensions(Obj,_,_,V),
  Out is In - 0.5*V.
center_part_pos(Obj, z, In, Out) :-
  object_dimensions(Obj,V,_,_),
  Out is In - 0.5*V.

belief_part_offset(Parent, PartType, Offset, Rotation) :-
  has_type(Parent, ParentType),

  %% Get Disposition
  subclass_of(ParentType, S),
  subclass_of(S, Description),
  is_restriction(Description, exactly(soma:'hasDisposition', 1, Disposition)),

  %% Get Trigger type
  subclass_of(Disposition, R), has_description(R, only(soma:'affordsTrigger', Desc)),  has_description(Desc, only(dul:'classifies', TriggerType)),
  transitive(subclass_of(PartType, TriggerType)),
  
  subclass_of(Disposition, SpaceRestriction), has_description(SpaceRestriction, value(soma:hasSpaceRegion, LinkageSpace)),
  holds(LinkageSpace, knowrob:quaternion, Q),
  holds(LinkageSpace, knowrob:translation, T),
  
  atomic_list_concat(T1,' ', T), maplist(atom_number, T1, Offset),
  atomic_list_concat(R1,' ', Q),  maplist(atom_number, R1, Rotation).

belief_part_offset(_, _, [0,0,0], [0,0,0,1]).

perceived_part_at_axis__(Parent, PartType, norm(Axis,Pos), Part) :- !,
  denormalize_part_pos(Parent, Axis, Pos, Denormalized), 
  perceived_part_at_axis__(Parent, PartType, pos(Axis,Denormalized), Part).

perceived_part_at_axis__(Parent, PartType, pos(Axis,Pos), Part) :- 
  center_part_pos(Parent, Axis, Pos, Centered),
  holds(Parent, knowrob:frameName, ParentFrame),
  once(belief_part_offset(Parent, PartType, Offset, Rotation)), 
  perceived_pos__(Offset, pos(Axis,Centered), PerceivedPos),
  tell(instance_of(Part,PartType)),
  tell(holds(Parent, soma:hasPhysicalComponent, Part)),
  tell(is_at(Part,[ParentFrame, PerceivedPos, Rotation])).
 

%%
%%

belief_shelf_barcode_at(Layer, Type, dan(DAN), PosNorm, Obj) :-
  belief_shelf_barcode_at(Layer, Type, dan(DAN), PosNorm, Obj, [insert,update_facings]).

belief_shelf_barcode_at(Layer, Type, dan(DAN), PosNorm, Obj, Options) :-
  belief_shelf_part_at(Layer, Type, PosNorm, Obj, Options),
  % 
  forall( article_number_of_dan(DAN,AN),
          tell(holds(Obj, shop:articleNumberOfLabel, AN)) ).

pos_term(Axis, norm(Pos), norm(Axis,Pos)) :- !.
pos_term(Axis, Pos, pos(Axis,Pos)).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'Product' 

product_dimensions([D,W,H], [D,W,H]) :- !.

product_dimensions(X,[D,W,H]):- 
  (object_dimensions(X, D, W, H));
  (subclass_of(X, R1), has_description(R1, value(shop:'depthOfProduct', D)),
  subclass_of(X, R2), has_description(R2, value(shop:'widthOfProduct', W)),
  subclass_of(X, R3), has_description(R3, value(shop:'heightOfProduct', H))), !.
product_dimensions(Type, [0.04,0.04,0.04]) :-
  print_message(warning, shop(Type,'No bounding box is defined')).

product_spawn_at(Facing, TypeOrBBOX, Offset_D, Obj) :-
  triple(Facing, shop:layerOfFacing, Layer),
  (triple(Facing, shop:'labelOfFacing', Label); triple(Facing, shop:'adjacentLabelOfFacing', Label)),
  triple(Label, shop:articleNumberOfLabel, ArticleNumber),

  product_dimensions(TypeOrBBOX, [Obj_D,_,Obj_H]),
  object_dimensions(Layer,Layer_D,_,_),
  Layer_D*0.5 > Offset_D + Obj_D*0.5 + 0.04,
  ( TypeOrBBOX=[D,W,H] -> (  
    belief_new_object(shop:'Product', Obj),
    tell(has_type(ProductShape, soma:'Shape')),
    tell(triple(Obj, soma:'hasShape', Productshape)),
    tell(object_dimensions(Obj, D, W, H)));
    belief_new_object(TypeOrBBOX, Obj) ),
  % enforce we have a product here   ASK: Below 3 lines, is it necessary
  ( instance_of(Obj,shop:'Product') -> true ;(
    print_message(warning, shop([Obj], 'Is not subclass of shop:Product.')),
    tell(has_type(Obj,shop:'Product')) )),
  comp_facingPose(Facing, [_,[Facing_X,_, _], _]),
  %is_at(Facing, [_,[Facing_X,_,_],_]), 
  % FIXME: this should be handled by offsets from ontology
  ( shelf_layer_standing(Layer) -> (
    shelf_layer_standing_bottom(Layer) ->
    Offset_H is  Obj_H*0.5 + 0.025 ;
    Offset_H is  Obj_H*0.5 + 0.08 ) ;
    Offset_H is -Obj_H*0.5 - 0.025 ),
  
  % HACK rotate if it has a mesh
  ( % object_mesh_path(Obj,_) CHECK
    transitive(subclass_of(TypeOrBBOX, Restriction)), has_description(Restriction, value(knowrob:'pathToCadModel', Value)) ->  
    Rot=[0.0, 0.0, 1.0, 0.0] ;
    Rot=[0.0, 0.0, 0.0, 1.0] ),
  %Rot=[0.0, 0.0, 0.0, 1.0],
  
  % declare transform
  holds(Layer, knowrob:frameName, LayerFrame),
  once(assert_object_shape_(Obj)),
  tell(is_at(Obj, [LayerFrame, 
      [Facing_X, Offset_D, Offset_H],
      Rot])),
  tell(holds(Facing, shop:productInFacing, Obj)),
  
  % update facing
  comp_mainColorOfFacing(Facing,Color),
  nth0(3, Color, _, FacingColor),
  tell(holds(Facing, knowrob:'mainColorOfObject', FacingColor)),
  marker_plugin:republish.
  % show_marker(Facing, Facing).

product_spawn_front_to_back(Facing, Obj) :-
  shelf_facing_product_type(Facing, ProductType),
  product_spawn_front_to_back(Facing, Obj, ProductType).
  
  
product_spawn_front_to_back(Facing, Obj, TypeOrBBOX) :-
  triple(Facing, shop:layerOfFacing, Layer),
  product_dimensions(TypeOrBBOX, [Obj_D,_,_]),
  shelf_facing_products(Facing, ProductsFrontToBack),
  reverse(ProductsFrontToBack, ProductsBackToFront),
  ( ProductsBackToFront=[] -> (
    object_dimensions(Layer,Layer_D,_,_),
    Obj_Pos is -Layer_D*0.5 + Obj_D*0.5 + 0.01,
    product_spawn_at(Facing, TypeOrBBOX, Obj_Pos, Obj));(
    ProductsBackToFront=[(Last_Pos,Last)|_],
    has_type(Last, LastType),
    product_dimensions(LastType,[Last_D,_,_]),
    Obj_Pos is Last_Pos + 0.5*Last_D + 0.5*Obj_D + 0.02,
    product_spawn_at(Facing, TypeOrBBOX, Obj_Pos, Obj)
  )), !.
  
shelf_facing_products(Facing, Products) :-
  findall((Pos,Product), (
    triple(Facing, shop:productInFacing, Product),
    is_at(Product, [_, [_,Pos,_], _])
    % current_object_pose(Product, [_,_,[_,Pos,_],_])
    ), Products_unsorted),
  sort(Products_unsorted, Products).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Helper predicates

max_negative_element([(_,D_A)|Xs], (Needle,D_Needle)) :-
  D_A > 0.0, !, max_negative_element(Xs, (Needle,D_Needle)).
max_negative_element([(A,D_A)|Rest], (Needle,D_Needle)) :-
  max_negative_element(Rest, (B,D_B)),
  ( D_A > D_B -> (
    Needle=A, D_Needle=D_A );(
    Needle=B, D_Needle=D_B )).
max_negative_element([(A,D_A)|_], (A,D_A)).

min_positive_element([(_,D_A)|Xs], (Needle,D_Needle)) :-
  D_A < 0.0, !, min_positive_element(Xs, (Needle,D_Needle)).
min_positive_element([(A,D_A)|Rest], (Needle,D_Needle)) :-
  min_positive_element(Rest, (B,D_B)),
  ( D_A < D_B ->
    Needle=A, D_Needle=D_A;
    Needle=B, D_Needle=D_B ).
min_positive_element([(A,D_A)|_], (A,D_A)).

rdfs_classify(Entity,Type) :-
  forall((
    triple(Entity,rdf:type,X),
    transitive(subclass_of(Type,X))),
    tripledb_forget(Entity,rdf:type,X)),
  tell(has_type(Entity,Type)).

owl_classify(Entity,Type) :-
  % find RDF graph of Entity
  forall((
    triple(Entity,rdf:type,X),
    once(subclass_of(Type,X))),
    tripledb_forget(Entity,rdf:type,X)),
  tell(has_type(Entity,Type)).

belief_new_object(ObjType, Obj) :-
  tell(instance_of(Obj, ObjType)),
  rdf_split_url(_, ObjFrameName, Obj), 
  tell(holds(Obj, knowrob:frameName, ObjFrameName)).

assert_object_shape_(Object):-  %% TODO : Check if the object shape of facing and labels are asserted, assert as box shape
  has_type(Object, ObjectType),
  
  ((triple(Object, soma:'Shape', Shape),
  holds(Shape,dul:hasRegion,ShapeRegion));
  (tell(has_type(Shape, soma:'Shape')),
  tell(holds(Object,soma:hasShape,Shape)),
  tell(has_type(ShapeRegion, soma:'ShapeRegion')),
  tell(holds(Shape,dul:hasRegion,ShapeRegion)))),

  transitive(subclass_of(ObjectType, R1)), has_description(R1, value(knowrob:pathToCadModel, FilePath)),
  tell(triple(ShapeRegion,soma:hasFilePath,FilePath)),
  Pos = [0,0,0], Rot = [0,0,0,1],

  tell(is_individual(Origin)),
  tell(triple(ShapeRegion,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin)),
	tell(triple(Origin, soma:hasPositionVector, term(Pos))),
	tell(triple(Origin, soma:hasOrientationVector, term(Rot))).

assert_object_shape_(Object, D, W, H, RGBValue):- 
  (object_dimensions(Object, D, W, H) -> true;
  tell(has_type(Shape, soma:'Shape')),
  tell(holds(Object,soma:hasShape,Shape)),
  tell(object_dimensions(Object, D, W, H))),
  holds(Shape,dul:hasRegion,ShapeRegion),

  Pos = [0,0,0], Rot = [0,0,0,1],
  tell(is_individual(Origin)),
  tell(triple(ShapeRegion,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin)),
	tell(triple(Origin, soma:hasPositionVector, term(Pos))),
	tell(triple(Origin, soma:hasOrientationVector, term(Rot))),

  tell(has_type(ColorType, soma:'Color')),
  tell(holds(Object,soma:hasColor,ColorType)),
  tell(object_color_rgb(Object, RGBValue)).

assert_perception_feature_(Object) :-
  once(assert_perception_feature__(Object)), !.

assert_perception_feature__(Object) :-
  has_type(Object, ObjectType),
  subclass_of(ObjectType, R), is_restriction(R, exactly(soma:hasFeature, 1, PerceptionFeature)),
  subclass_of(PerceptionFeature, Desc), has_description(Desc, value(knowrob:pose, FeaturePose)),
  
  holds(FeaturePose, knowrob:translation, Translation),
  holds(FeaturePose, knowrob:quaternion, Rotation),

  atomic_list_concat(T1,' ', Translation),  maplist(atom_number, T1, T2), 
  atomic_list_concat(R1,' ', Rotation),  maplist(atom_number, R1, R2), 
  
  tell(has_type(FeatureIndividual, PerceptionFeature)),
  tell(triple(Object, soma:hasFeature, FeatureIndividual)),
  
  holds(Object, knowrob:frameName, ObjFrame),
  tell(is_at(FeatureIndividual, [ObjFrame, T2, R2])),
  rdf_split_url(_, FeatureFrameName, FeatureIndividual), 
  tell(holds(FeatureIndividual, knowrob:frameName, FeatureFrameName)).
