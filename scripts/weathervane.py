windref = {}
windref[38]="West"
windref[67]="NW"
windref[100]="WNW"
windref[131]="North"
windref[182]="NNW"
windref[238]="SW"
windref[261]="WSW"
windref[379]="NE"
windref[433]="NNE"
windref[554]="South"
windref[611]="SSW"
windref[697]="SE"
windref[786]="SSE"
windref[844]="East"
windref[860]="ENE"
windref[892]="ESE"

def direction(number):
    for i in windref.keys():
        if i-8 < number < i+8 :
            return windref[i]
    return "Unknown"
