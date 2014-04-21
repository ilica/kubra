// Foundation JavaScript
// Documentation can be found at: http://foundation.zurb.com/docs

$(document).foundation();


$("#whatItIsButton").click(function() {
    var modal = $('#whatItIs');
    modal.foundation('reveal', 'open');
    modal.css('width', $('body').width());
    draw();
});

$("#howItWorksButton").click(function() {
    var modal = $('#howItWorks');
    modal.foundation('reveal', 'open');
    modal.css('width', $('body').width());
});

$("#whosBehindItButton").click(function() {
    var modal = $('#whosBehindIt');
    modal.foundation('reveal', 'open');
    modal.css('width', $('body').width());
});


$(".full-modal").click(function() {
    $(this).foundation('reveal', 'close');
});
