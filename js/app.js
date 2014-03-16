// Foundation JavaScript
// Documentation can be found at: http://foundation.zurb.com/docs


$(document).foundation();


console.log("HELLO");

$("#content").click( function() {
    $('#myModal').foundation('reveal', 'open');
});
