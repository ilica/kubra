// Foundation JavaScript
// Documentation can be found at: http://foundation.zurb.com/docs


$(document).foundation();

$("#content").click(function() {
    $('#myModal').foundation('reveal', 'open');
});

$('#Logo').hide();

$(document).ready( function() {
  $('#Logo').fadeIn(4000);
});

