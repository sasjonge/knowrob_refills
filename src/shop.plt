:- use_module(library('lang/terms/is_a')).
:- use_module(library('lang/terms/holds')).
:- use_module(library('lang/query')).
:- use_module(library('shop')).


:- rdf_db:rdf_register_ns(dmshop, 'http://knowrob.org/kb/dm-market.owl#', [keep(true)]).

:- tripledb:tripledb_load(
    'http://www.ease-crc.org/ont/SOMA.owl',
    [ namespace(soma, 
      'http://www.ease-crc.org/ont/SOMA.owl#')
    ]).

:- tripledb_load('package://knowrob_refills/owl/shop-test.owl', 
  [namespace(shoptest,'http://knowrob.org/kb/shop-test.owl#')] ).

:- begin_tripledb_tests(
    shop,
    'package://knowrob_refills/owl/shop.owl',
    [ namespace('http://knowrob.org/kb/shop.owl#')
    ]).

    %   shelf_classify(r,+,+,+),
    %   shelf_with_marker(r,r),
    %   %shelf_estimate_pose/1,
    %   %%%%%
    %   belief_shelf_part_at(r,r,+,r),
    %   belief_shelf_part_at(r,r,+,r,+),
    %   belief_shelf_barcode_at(r,r,+,+,-),
    %   belief_shelf_barcode_at(r,r,+,+,-,+),
    %   product_spawn_front_to_back(r,r),
    %   product_spawn_front_to_back(r,r,r),

test('create an article number'):-
    X = 'GTIN_2344768595340257',
    GT = '2344768590257',
    DA = '786453',
    create_article_number(GT, DA, X),
    instance_of(X, test:'ArticleNumber').

test('create a shelf left_marker at a pose') :-
    MarkerPoseData = ['Frame1', [1.0,0.4,2.32], [0.0,0.0,0.0,1.0]],
    MarkerId = 'tag_85',
    Marker = 'DMShelfMarkerLeft_BZXDHRWP',
    belief_shelf_left_marker_at(MarkerPoseData, MarkerId, Marker),
    holds(Marker, dmshop:'markerId', X) -> assert_equals(X, MarkerId).
    
test('create a shelf right marker at a pose') :-
    MarkerPoseData = ['Frame1', [1.0,0.4,2.32], [0.0,0.0,0.0,1.0]],
    MarkerId = 'tag_84',
    Marker = 'DMShelfMarkerRight_BZXDHRWP',
    belief_shelf_right_marker_at(MarkerPoseData, MarkerId, Marker),
    holds(Marker, dmshop:'markerId', X) -> assert_equals(X, MarkerId).

test('create a shelf with left and right marker') :-
    LeftMarkerPoseData = [map, [1.2621, -1.1481, 0.0609], [0.0082, 0.0145, 0.9961, 0.08570]],
    LeftMarkerId = 'tag_39',
    LeftMarker = 'DMShelfMarkerLeft_GKTIENZH',
    belief_shelf_left_marker_at(LeftMarkerPoseData, LeftMarkerId, LeftMarker),
    assert_true(is_at(LeftMarker, LeftMarkerPoseData)),
    
    RightMarkerPoseData = [map, [0.3198, -1.1169, 0.04958], [0.00146, -0.02363, 0.9987, -0.04307]],
    RightMarkerId = 'tag_80',
    RightMarker = 'DMShelfMarkerRight_NJZRGQCW',
    belief_shelf_right_marker_at(RightMarkerPoseData, RightMarkerId, RightMarker),
    
    Shelf = 'DMShelfW100_LOMKBJCG',
    belief_shelf_at(LeftMarkerId,RightMarkerId,Shelf),

    holds(Shelf, knowrob:'depthOfObject', Depth) -> assert_equals(Depth, 0.02).

test('create a shelf part at') :-
    %Frame = shoptest:'DMShelfSystem_He63Gwp6',
    % instance_of(Frame, FrameType), ASK SASCHA strange error
    %instance_of(shoptest:'DMShelfSystem_He63Gwp6', FrameType),
    tell(subclass_of(shoptest:'DMShelfLayer_test', dmshop:'DMShelfFloor')),
    belief_shelf_part_at(shoptest:'DMShelfLayer_test', dmshop:'DMShelfFloor', '0.7', _).

test('debug cycle') :-
    gtrace,
    tell(has_type(X, dmshop:'DMBFloorT7W100')),
    rdf_split_url(_, ObjFrameName, X), 
    tell(holds(X, knowrob:frameName, ObjFrameName)),
    

    tell(instance_of(LSep, shop:'leftSeparator')),
    tell(instance_of(RSep, shop:'rightSeparator')),

    tell(has_type(Facing, shop:'ProductFacingStanding')),
    tell(holds(Facing, shop:'leftSeparator', LSep)),
    tell(is_at(LSep, [X, [2.3,4.5,0], [0,0,0,1]])),
    tell(holds(Facing, shop:'rightSeparator', RSep)),
    tell(is_at(RSep, [X, [4.3,4.5,0], [0,0,0,1]])),
    tell(holds(Facing, shop:layerOfFacing, X)),
    
    shelf_layer_find_facing_at(X, -0.029, Facing).

:- end_tripledb_tests(shop).